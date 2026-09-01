pub mod arduino_ota;
pub mod motor;
pub mod radio;
pub mod radio_calibration;
pub mod update;

use std::{
    fmt::Debug,
    io,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use arduino_ota::{ArduinoOta, ARDUINO_APPLICATION_LIMIT};
use embedded_svc::{
    http::{Headers, Method},
    io::{Read as EmbeddedRead, Write as EmbeddedWrite},
    wifi::{
        AccessPointConfiguration, AuthMethod, ClientConfiguration,
        Configuration as WifiConfiguration,
    },
};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::{
        gpio::AnyIOPin,
        peripherals::Peripherals,
        reset,
        uart::{config::Config as UartConfiguration, UartDriver},
        units::Hertz,
    },
    http::server::{Configuration as HttpConfiguration, EspHttpServer},
    mdns::EspMdns,
    nvs::{EspDefaultNvs, EspDefaultNvsPartition, EspNvs},
    wifi::{BlockingWifi, EspWifi},
};
use motor::MotorService;
use radio::RadioService;
use radio_calibration::{RadioCalibration, MAX_CALIBRATION_FORM_SIZE};
use update::{UpdateManager, ESP32_OTA_SLOT_SIZE};

const SETUP_SSID: &str = "Wobot-Setup";
const SETUP_PASSWORD: &str = "wobot-update";
const INDEX_TEMPLATE: &str = include_str!("index.html");
const SETUP_TEMPLATE: &str = include_str!("setup.html");
const MAX_WIFI_FORM_SIZE: usize = 384;
const FIRMWARE_VERSION: &str = env!("CARGO_PKG_VERSION");

type CredentialsStore = Arc<Mutex<EspDefaultNvs>>;

#[derive(Debug, Clone)]
struct WifiCredentials {
    ssid: String,
    password: String,
}

enum NetworkMode {
    Station {
        ssid: String,
        address: String,
    },
    Setup {
        networks: Vec<String>,
        previous_connection_failed: bool,
    },
}

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let pins = peripherals.pins;
    let arduino_uart = UartDriver::new(
        peripherals.uart2,
        pins.gpio17,
        pins.gpio18,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &UartConfiguration::default()
            .baudrate(Hertz(115_200))
            .tx_fifo_size(0),
    )?;
    let arduino_ota = Arc::new(Mutex::new(ArduinoOta::new(arduino_uart)?));
    let system_loop = EspSystemEventLoop::take()?;
    let nvs_partition = EspDefaultNvsPartition::take()?;
    let credentials_store = Arc::new(Mutex::new(EspNvs::new(
        nvs_partition.clone(),
        "wobot",
        true,
    )?));
    let stored_credentials = {
        let store = credentials_store
            .lock()
            .map_err(|_| anyhow::anyhow!("Wi-Fi credentials lock poisoned"))?;
        load_credentials(&store)?
    };
    let radio_calibration = {
        let store = credentials_store
            .lock()
            .map_err(|_| anyhow::anyhow!("calibration store lock poisoned"))?;
        radio_calibration::load(&store)?
    };
    let radio_calibration = Arc::new(Mutex::new(radio_calibration));

    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, system_loop.clone(), Some(nvs_partition))?,
        system_loop,
    )?;

    let network_mode = if let Some(credentials) = stored_credentials {
        match start_station(&mut wifi, &credentials) {
            Ok(address) => NetworkMode::Station {
                ssid: credentials.ssid,
                address,
            },
            Err(error) => {
                log::warn!("Could not connect to configured Wi-Fi: {error:#}");
                let _ = wifi.stop();
                let networks = start_setup_mode(&mut wifi)?;
                NetworkMode::Setup {
                    networks,
                    previous_connection_failed: true,
                }
            }
        }
    } else {
        let networks = start_setup_mode(&mut wifi)?;
        NetworkMode::Setup {
            networks,
            previous_connection_failed: false,
        }
    };

    let page = Arc::<str>::from(match &network_mode {
        NetworkMode::Station { ssid, address } => build_update_page(ssid, address),
        NetworkMode::Setup {
            networks,
            previous_connection_failed,
        } => build_setup_page(networks, *previous_connection_failed),
    });

    let radio = RadioService::start();
    let motors = MotorService::start(
        radio.clone(),
        Arc::clone(&radio_calibration),
        Arc::clone(&arduino_ota),
    );

    let server_configuration = HttpConfiguration {
        stack_size: 20 * 1024,
        ..Default::default()
    };
    let mut server = EspHttpServer::new(&server_configuration)?;

    let page_for_handler = Arc::clone(&page);
    server.fn_handler("/", Method::Get, move |request| {
        request
            .into_response(
                200,
                Some("OK"),
                &[
                    ("Content-Type", "text/html; charset=utf-8"),
                    ("Cache-Control", "no-store"),
                ],
            )?
            .write_all(page_for_handler.as_bytes())
            .map(|_| ())
    })?;

    register_ota_handler(&mut server, motors.clone())?;
    register_arduino_ota_handlers(&mut server, arduino_ota, motors.clone())?;
    register_status_handler(&mut server)?;
    register_motor_status_handler(&mut server, motors)?;
    register_wifi_handlers(&mut server, Arc::clone(&credentials_store))?;
    register_radio_handlers(
        &mut server,
        radio,
        Arc::clone(&credentials_store),
        radio_calibration,
    )?;

    let mut mdns = EspMdns::take()?;
    mdns.set_hostname("wobot")?;
    mdns.set_instance_name("Wobot Robot")?;
    mdns.add_service(
        Some("Wobot maintenance"),
        "_http",
        "_tcp",
        server_configuration.http_port,
        &[("path", "/")],
    )?;

    // NVS, networking, mDNS, and the maintenance server are alive. A pending
    // image is safe to confirm only after all of these startup checks pass.
    let mut update_manager = UpdateManager::new()?;
    update_manager.confirm_running_image()?;
    drop(update_manager);

    match &network_mode {
        NetworkMode::Station { ssid, address } => {
            log::info!("Wobot connected to {ssid}; http://{address} or http://wobot.local");
        }
        NetworkMode::Setup { .. } => {
            log::info!(
                "Configure Wi-Fi on {SETUP_SSID} (password {SETUP_PASSWORD}), http://192.168.71.1"
            );
        }
    }

    loop {
        thread::sleep(Duration::from_secs(60));
    }
}

fn register_radio_handlers(
    server: &mut EspHttpServer<'static>,
    radio: RadioService,
    calibration_store: CredentialsStore,
    calibration_state: Arc<Mutex<RadioCalibration>>,
) -> anyhow::Result<()> {
    let status_radio = radio.clone();
    server.fn_handler("/api/radio/status", Method::Get, move |request| {
        let snapshot = status_radio.snapshot();
        let transmitter = snapshot
            .transmitter
            .map(|address| format!("\"{address}\""))
            .unwrap_or_else(|| "null".to_owned());
        let age_ms = snapshot
            .age_ms
            .map(|age| age.to_string())
            .unwrap_or_else(|| "null".to_owned());
        let channels = snapshot.channels[..snapshot.channel_count]
            .iter()
            .map(u16::to_string)
            .collect::<Vec<_>>()
            .join(",");
        let error = snapshot
            .error
            .as_deref()
            .map(|error| format!("\"{}\"", escape_json(error)))
            .unwrap_or_else(|| "null".to_owned());
        let body = format!(
            "{{\"ok\":true,\"connected\":{},\"transmitter\":{transmitter},\"age_ms\":{age_ms},\"packets\":{},\"channels\":[{channels}],\"error\":{error}}}",
            snapshot.connected, snapshot.packet_count
        );
        request
            .into_response(
                200,
                Some("OK"),
                &[
                    ("Content-Type", "application/json"),
                    ("Cache-Control", "no-store"),
                ],
            )?
            .write_all(body.as_bytes())
            .map(|_| ())
    })?;

    server.fn_handler("/api/radio/activate", Method::Post, move |request| {
        radio.request_activation();
        request
            .into_response(
                202,
                Some("Accepted"),
                &[("Content-Type", "application/json")],
            )?
            .write_all(br#"{"ok":true,"activating":true}"#)
            .map(|_| ())
    })?;

    let get_calibration = Arc::clone(&calibration_state);
    server.fn_handler::<anyhow::Error, _>(
        "/api/radio/calibration",
        Method::Get,
        move |request| {
            let calibration = get_calibration
                .lock()
                .map_err(|_| anyhow::anyhow!("calibration state lock poisoned"))?
                .clone();
            request
                .into_response(
                    200,
                    Some("OK"),
                    &[
                        ("Content-Type", "application/json"),
                        ("Cache-Control", "no-store"),
                    ],
                )?
                .write_all(calibration.to_json().as_bytes())?;
            Ok(())
        },
    )?;

    let save_calibration = calibration_state;
    server.fn_handler::<anyhow::Error, _>(
        "/api/radio/calibration",
        Method::Post,
        move |mut request| {
            let content_len = request.content_len().unwrap_or(0) as usize;
            if content_len == 0 || content_len > MAX_CALIBRATION_FORM_SIZE {
                request
                    .into_status_response(400)?
                    .write_all(b"invalid calibration size")?;
                return Ok(());
            }
            let mut body = vec![0_u8; content_len];
            request.read_exact(&mut body)?;
            let calibration = match RadioCalibration::from_form(&body) {
                Ok(calibration) => calibration,
                Err(error) => {
                    log::warn!("Rejected radio calibration: {error:#}");
                    let body = format!(
                        "{{\"ok\":false,\"error\":\"invalid_calibration\",\"detail\":\"{}\"}}",
                        escape_json(&error.to_string())
                    );
                    request
                        .into_response(
                            400,
                            Some("Bad Request"),
                            &[("Content-Type", "application/json")],
                        )?
                        .write_all(body.as_bytes())?;
                    return Ok(());
                }
            };
            {
                let store = calibration_store
                    .lock()
                    .map_err(|_| anyhow::anyhow!("calibration store lock poisoned"))?;
                radio_calibration::save(&store, &calibration)?;
            }
            *save_calibration
                .lock()
                .map_err(|_| anyhow::anyhow!("calibration state lock poisoned"))? =
                calibration.clone();
            request
                .into_response(200, Some("OK"), &[("Content-Type", "application/json")])?
                .write_all(calibration.to_json().as_bytes())?;
            Ok(())
        },
    )?;
    Ok(())
}

fn register_arduino_ota_handlers(
    server: &mut EspHttpServer<'static>,
    updater: Arc<Mutex<ArduinoOta>>,
    motors: MotorService,
) -> anyhow::Result<()> {
    let staging_updater = Arc::clone(&updater);
    let staging_motors = motors.clone();
    server.fn_handler::<anyhow::Error, _>(
        "/api/ota/arduino/stage",
        Method::Post,
        move |mut request| {
            staging_motors.require_disarm();
            let payload_len = request.content_len().unwrap_or(0) as usize;
            if payload_len == 0 || payload_len > ARDUINO_APPLICATION_LIMIT {
                request
                    .into_response(
                        413,
                        Some("Payload Too Large"),
                        &[("Content-Type", "application/json")],
                    )?
                    .write_all(br#"{"ok":false,"error":"invalid_size"}"#)?;
                return Ok(());
            }

            let result = {
                let mut updater = staging_updater
                    .lock()
                    .map_err(|_| anyhow::anyhow!("Arduino OTA lock poisoned"))?;
                let mut reader = StdReadAdapter(&mut request);
                updater.stage(payload_len, &mut reader)
            };

            match result {
                Ok(()) => request
                    .into_response(200, Some("OK"), &[("Content-Type", "application/json")])?
                    .write_all(format!(r#"{{"ok":true,"staged":{payload_len}}}"#).as_bytes())?,
                Err(error) => {
                    log::error!("Arduino staging failed: {error:?}");
                    request
                        .into_response(
                            400,
                            Some("Bad Request"),
                            &[("Content-Type", "application/json")],
                        )?
                        .write_all(br#"{"ok":false,"error":"staging_failed"}"#)?;
                }
            }
            Ok(())
        },
    )?;

    server.fn_handler::<anyhow::Error, _>(
        "/api/ota/arduino/flash",
        Method::Post,
        move |request| {
            if request.header("X-Wobot-Action") != Some("flash-arduino") {
                request.into_status_response(403)?.write_all(b"forbidden")?;
                return Ok(());
            }

            let _maintenance = motors.maintenance();
            // Let the AVR watchdog remove motor power and let the final motor
            // frame leave the UART FIFO before Optiboot takes over the line.
            thread::sleep(Duration::from_millis(200));
            let result = {
                let mut updater = updater
                    .lock()
                    .map_err(|_| anyhow::anyhow!("Arduino OTA lock poisoned"))?;
                updater.flash()
            };
            match result {
                Ok(()) => request
                    .into_response(200, Some("OK"), &[("Content-Type", "application/json")])?
                    .write_all(br#"{"ok":true,"verified":true}"#)?,
                Err(error) => {
                    log::error!("Arduino programming failed: {error:?}");
                    let detail = format!("{error:?}")
                        .replace('\\', "\\\\")
                        .replace('"', "\\\"")
                        .replace('\n', "\\n");
                    request
                        .into_response(
                            409,
                            Some("Conflict"),
                            &[("Content-Type", "application/json")],
                        )?
                        .write_all(
                            format!(
                                r#"{{"ok":false,"error":"programming_failed","detail":"{detail}"}}"#
                            )
                            .as_bytes(),
                        )?;
                }
            }
            Ok(())
        },
    )?;
    Ok(())
}

fn register_status_handler(server: &mut EspHttpServer<'static>) -> anyhow::Result<()> {
    server.fn_handler("/api/status", Method::Get, |request| {
        let body = format!(
            r#"{{"ok":true,"device":"wobot","target":"esp32s3","board":"yd-esp32-s3-n16r8","version":"{FIRMWARE_VERSION}"}}"#,
        );
        request
            .into_response(
                200,
                Some("OK"),
                &[
                    ("Content-Type", "application/json"),
                    ("Cache-Control", "no-store"),
                ],
            )?
            .write_all(body.as_bytes())
            .map(|_| ())
    })?;
    Ok(())
}

fn register_motor_status_handler(
    server: &mut EspHttpServer<'static>,
    motors: MotorService,
) -> anyhow::Result<()> {
    server.fn_handler("/api/drive/status", Method::Get, move |request| {
        let body = motor::status_json(&motors.snapshot());
        request
            .into_response(
                200,
                Some("OK"),
                &[
                    ("Content-Type", "application/json"),
                    ("Cache-Control", "no-store"),
                ],
            )?
            .write_all(body.as_bytes())
            .map(|_| ())
    })?;
    Ok(())
}

fn register_ota_handler(
    server: &mut EspHttpServer<'static>,
    motors: MotorService,
) -> anyhow::Result<()> {
    server.fn_handler::<anyhow::Error, _>("/api/ota/esp32", Method::Post, move |mut request| {
        motors.require_disarm();
        let payload_len = request.content_len().unwrap_or(0) as usize;
        if payload_len == 0 || payload_len > ESP32_OTA_SLOT_SIZE {
            request
                .into_response(
                    413,
                    Some("Payload Too Large"),
                    &[("Content-Type", "application/json")],
                )?
                .write_all(br#"{"ok":false,"error":"invalid_size"}"#)?;
            return Ok(());
        }

        let result = {
            let mut reader = StdReadAdapter(&mut request);
            let mut manager = UpdateManager::new()?;
            manager.install_esp32(payload_len, &mut reader)
        };

        match result {
            Ok(()) => {
                request
                    .into_response(200, Some("OK"), &[("Content-Type", "application/json")])?
                    .write_all(br#"{"ok":true,"rebooting":true}"#)?;
                schedule_restart()?;
            }
            Err(error) => {
                log::error!("OTA failed: {error:?}");
                request
                    .into_response(
                        400,
                        Some("Bad Request"),
                        &[("Content-Type", "application/json")],
                    )?
                    .write_all(br#"{"ok":false,"error":"update_failed"}"#)?;
            }
        }
        Ok(())
    })?;
    Ok(())
}

fn register_wifi_handlers(
    server: &mut EspHttpServer<'static>,
    credentials_store: CredentialsStore,
) -> anyhow::Result<()> {
    let configure_store = Arc::clone(&credentials_store);
    server.fn_handler::<anyhow::Error, _>(
        "/api/wifi/configure",
        Method::Post,
        move |mut request| {
            let content_len = request.content_len().unwrap_or(0) as usize;
            if content_len == 0 || content_len > MAX_WIFI_FORM_SIZE {
                request
                    .into_status_response(400)?
                    .write_all(b"invalid form size")?;
                return Ok(());
            }

            let mut body = vec![0_u8; content_len];
            request.read_exact(&mut body)?;
            let credentials = match parse_wifi_form(&body) {
                Ok(credentials) => credentials,
                Err(error) => {
                    log::warn!("Rejected Wi-Fi configuration: {error:#}");
                    request
                        .into_status_response(400)?
                        .write_all(b"invalid Wi-Fi configuration")?;
                    return Ok(());
                }
            };

            {
                let store = configure_store
                    .lock()
                    .map_err(|_| anyhow::anyhow!("Wi-Fi credentials lock poisoned"))?;
                save_credentials(&store, &credentials)?;
            }

            request
                .into_response(200, Some("OK"), &[("Content-Type", "application/json")])?
                .write_all(br#"{"ok":true,"rebooting":true}"#)?;
            schedule_restart()?;
            Ok(())
        },
    )?;

    let reset_store = credentials_store;
    server.fn_handler::<anyhow::Error, _>("/api/wifi/reset", Method::Post, move |request| {
        if request.header("X-Wobot-Action") != Some("reset-wifi") {
            request.into_status_response(403)?.write_all(b"forbidden")?;
            return Ok(());
        }

        {
            let store = reset_store
                .lock()
                .map_err(|_| anyhow::anyhow!("Wi-Fi credentials lock poisoned"))?;
            clear_credentials(&store)?;
        }
        request
            .into_response(200, Some("OK"), &[("Content-Type", "application/json")])?
            .write_all(br#"{"ok":true,"rebooting":true}"#)?;
        schedule_restart()?;
        Ok(())
    })?;

    Ok(())
}

fn start_station(
    wifi: &mut BlockingWifi<EspWifi<'static>>,
    credentials: &WifiCredentials,
) -> anyhow::Result<String> {
    let auth_method = if credentials.password.is_empty() {
        AuthMethod::None
    } else {
        AuthMethod::WPA2WPA3Personal
    };
    let configuration = WifiConfiguration::Client(ClientConfiguration {
        ssid: credentials
            .ssid
            .as_str()
            .try_into()
            .map_err(|_| anyhow::anyhow!("SSID is too long"))?,
        password: credentials
            .password
            .as_str()
            .try_into()
            .map_err(|_| anyhow::anyhow!("Wi-Fi password is too long"))?,
        auth_method,
        ..Default::default()
    });

    wifi.set_configuration(&configuration)?;
    wifi.start()?;
    wifi.connect()?;
    wifi.wait_netif_up()?;
    Ok(wifi.wifi().sta_netif().get_ip_info()?.ip.to_string())
}

fn start_setup_mode(wifi: &mut BlockingWifi<EspWifi<'static>>) -> anyhow::Result<Vec<String>> {
    let networks = scan_networks(wifi);
    let configuration = WifiConfiguration::AccessPoint(AccessPointConfiguration {
        ssid: SETUP_SSID.try_into().expect("valid setup SSID"),
        password: SETUP_PASSWORD.try_into().expect("valid setup password"),
        auth_method: AuthMethod::WPA2Personal,
        channel: 6,
        max_connections: 2,
        ..Default::default()
    });
    wifi.set_configuration(&configuration)?;
    wifi.start()?;
    wifi.wait_netif_up()?;
    Ok(networks)
}

fn scan_networks(wifi: &mut BlockingWifi<EspWifi<'static>>) -> Vec<String> {
    let result = (|| -> anyhow::Result<Vec<String>> {
        wifi.set_configuration(&WifiConfiguration::Client(ClientConfiguration::default()))?;
        wifi.start()?;
        let access_points = wifi.scan()?;
        wifi.stop()?;

        let mut names: Vec<String> = access_points
            .into_iter()
            .map(|access_point| access_point.ssid.to_string())
            .filter(|ssid| !ssid.is_empty())
            .collect();
        names.sort();
        names.dedup();
        Ok(names)
    })();

    match result {
        Ok(networks) => networks,
        Err(error) => {
            log::warn!("Wi-Fi scan failed: {error:#}");
            let _ = wifi.stop();
            Vec::new()
        }
    }
}

fn load_credentials(store: &EspDefaultNvs) -> anyhow::Result<Option<WifiCredentials>> {
    if store.get_u8("wifi_valid")? != Some(1) {
        return Ok(None);
    }

    let mut ssid_buffer = [0_u8; 33];
    let mut password_buffer = [0_u8; 65];
    let Some(ssid) = store.get_str("wifi_ssid", &mut ssid_buffer)? else {
        return Ok(None);
    };
    let Some(password) = store.get_str("wifi_pass", &mut password_buffer)? else {
        return Ok(None);
    };

    Ok(Some(WifiCredentials {
        ssid: ssid.to_owned(),
        password: password.to_owned(),
    }))
}

fn save_credentials(store: &EspDefaultNvs, credentials: &WifiCredentials) -> anyhow::Result<()> {
    store.set_u8("wifi_valid", 0)?;
    store.set_str("wifi_ssid", &credentials.ssid)?;
    store.set_str("wifi_pass", &credentials.password)?;
    store.set_u8("wifi_valid", 1)?;
    Ok(())
}

fn clear_credentials(store: &EspDefaultNvs) -> anyhow::Result<()> {
    store.set_u8("wifi_valid", 0)?;
    let _ = store.remove("wifi_ssid")?;
    let _ = store.remove("wifi_pass")?;
    Ok(())
}

fn parse_wifi_form(body: &[u8]) -> anyhow::Result<WifiCredentials> {
    let form = std::str::from_utf8(body)?;
    let mut ssid = None;
    let mut password = None;

    for field in form.split('&') {
        let (key, value) = field
            .split_once('=')
            .ok_or_else(|| anyhow::anyhow!("malformed form field"))?;
        match key {
            "ssid" => ssid = Some(percent_decode(value)?),
            "password" => password = Some(percent_decode(value)?),
            _ => {}
        }
    }

    let ssid = ssid.ok_or_else(|| anyhow::anyhow!("SSID is missing"))?;
    let password = password.unwrap_or_default();
    if ssid.is_empty() || ssid.len() > 32 {
        anyhow::bail!("SSID must contain 1 to 32 bytes");
    }
    if password.len() > 63 {
        anyhow::bail!("password must contain at most 63 bytes");
    }
    if ssid.contains('\0') || password.contains('\0') {
        anyhow::bail!("NUL is not allowed");
    }

    Ok(WifiCredentials { ssid, password })
}

fn percent_decode(value: &str) -> anyhow::Result<String> {
    let bytes = value.as_bytes();
    let mut decoded = Vec::with_capacity(bytes.len());
    let mut index = 0;
    while index < bytes.len() {
        match bytes[index] {
            b'+' => decoded.push(b' '),
            b'%' => {
                if index + 2 >= bytes.len() {
                    anyhow::bail!("incomplete percent escape");
                }
                decoded.push((hex_value(bytes[index + 1])? << 4) | hex_value(bytes[index + 2])?);
                index += 2;
            }
            byte => decoded.push(byte),
        }
        index += 1;
    }
    Ok(String::from_utf8(decoded)?)
}

fn hex_value(byte: u8) -> anyhow::Result<u8> {
    match byte {
        b'0'..=b'9' => Ok(byte - b'0'),
        b'a'..=b'f' => Ok(byte - b'a' + 10),
        b'A'..=b'F' => Ok(byte - b'A' + 10),
        _ => anyhow::bail!("invalid percent escape"),
    }
}

fn build_update_page(ssid: &str, address: &str) -> String {
    let status = format!(
        "Sieć <code>{}</code> · <code>{}</code>",
        escape_html(ssid),
        escape_html(address)
    );
    INDEX_TEMPLATE
        .replace("{{NETWORK_STATUS}}", &status)
        .replace("{{FIRMWARE_VERSION}}", FIRMWARE_VERSION)
}

fn build_setup_page(networks: &[String], previous_connection_failed: bool) -> String {
    let options = networks
        .iter()
        .map(|ssid| format!("<option value=\"{}\"></option>", escape_html(ssid)))
        .collect::<String>();
    let notice = if previous_connection_failed {
        "<div class=\"notice\"><strong>Nie udało się połączyć z zapisaną siecią.</strong><p>Wybierz sieć ponownie i sprawdź hasło.</p></div>"
    } else {
        ""
    };
    SETUP_TEMPLATE
        .replace("{{NETWORK_OPTIONS}}", &options)
        .replace("{{CONNECTION_NOTICE}}", notice)
}

fn escape_html(value: &str) -> String {
    let mut escaped = String::with_capacity(value.len());
    for character in value.chars() {
        match character {
            '&' => escaped.push_str("&amp;"),
            '<' => escaped.push_str("&lt;"),
            '>' => escaped.push_str("&gt;"),
            '"' => escaped.push_str("&quot;"),
            '\'' => escaped.push_str("&#39;"),
            _ => escaped.push(character),
        }
    }
    escaped
}

fn escape_json(value: &str) -> String {
    let mut escaped = String::with_capacity(value.len());
    for character in value.chars() {
        match character {
            '"' => escaped.push_str("\\\""),
            '\\' => escaped.push_str("\\\\"),
            '\n' => escaped.push_str("\\n"),
            '\r' => escaped.push_str("\\r"),
            '\t' => escaped.push_str("\\t"),
            character if character.is_control() => escaped.push('?'),
            _ => escaped.push(character),
        }
    }
    escaped
}

fn schedule_restart() -> anyhow::Result<()> {
    thread::Builder::new()
        .name("wobot-restart".into())
        .stack_size(2048)
        .spawn(|| {
            thread::sleep(Duration::from_millis(900));
            reset::restart();
        })?;
    Ok(())
}

struct StdReadAdapter<'a, R>(&'a mut R);

impl<R> io::Read for StdReadAdapter<'_, R>
where
    R: EmbeddedRead,
    R::Error: Debug,
{
    fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        EmbeddedRead::read(self.0, buffer).map_err(|error| io::Error::other(format!("{error:?}")))
    }
}
