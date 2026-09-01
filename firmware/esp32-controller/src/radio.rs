use std::{
    io::{Read, Write},
    net::{IpAddr, SocketAddr, TcpStream, ToSocketAddrs, UdpSocket},
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

use wobot_core::ElrsChannels;

pub const JOYSTICK_PORT: u16 = 11_000;
const UPDATE_INTERVAL_US: u32 = 10_000;
const CHANNEL_COUNT: u8 = 10;
const RECEIVE_TIMEOUT: Duration = Duration::from_millis(500);
const PACKET_STALE_AFTER: Duration = Duration::from_secs(1);
const REACTIVATE_AFTER: Duration = Duration::from_secs(2);

#[derive(Debug, Clone)]
pub struct RadioSnapshot {
    pub connected: bool,
    pub transmitter: Option<IpAddr>,
    pub channels: [u16; ElrsChannels::MAX_CHANNELS],
    pub channel_count: usize,
    pub packet_count: u64,
    pub age_ms: Option<u64>,
    pub error: Option<String>,
}

#[derive(Debug)]
struct RadioState {
    transmitter: Option<IpAddr>,
    channels: [u16; ElrsChannels::MAX_CHANNELS],
    channel_count: usize,
    packet_count: u64,
    last_packet: Option<Instant>,
    force_activation: bool,
    error: Option<String>,
}

impl Default for RadioState {
    fn default() -> Self {
        Self {
            transmitter: None,
            channels: [0; ElrsChannels::MAX_CHANNELS],
            channel_count: 0,
            packet_count: 0,
            last_packet: None,
            force_activation: true,
            error: None,
        }
    }
}

#[derive(Clone)]
pub struct RadioService {
    state: Arc<Mutex<RadioState>>,
}

impl RadioService {
    pub fn start() -> Self {
        let service = Self {
            state: Arc::new(Mutex::new(RadioState::default())),
        };
        let worker_state = Arc::clone(&service.state);
        if let Err(error) = thread::Builder::new()
            .name("wobot-elrs".into())
            .stack_size(10 * 1024)
            .spawn(move || {
                if let Err(error) = receiver_loop(&worker_state) {
                    set_error(&worker_state, format!("Odbiornik ELRS zatrzymany: {error}"));
                }
            })
        {
            set_error(
                &service.state,
                format!("Nie udało się uruchomić odbiornika ELRS: {error}"),
            );
        }
        service
    }

    pub fn snapshot(&self) -> RadioSnapshot {
        let state = self.state.lock().expect("radio state lock poisoned");
        let age = state.last_packet.map(|seen| seen.elapsed());
        RadioSnapshot {
            connected: age.is_some_and(|age| age <= PACKET_STALE_AFTER),
            transmitter: state.transmitter,
            channels: state.channels,
            channel_count: state.channel_count,
            packet_count: state.packet_count,
            age_ms: age.map(|age| age.as_millis().min(u64::MAX as u128) as u64),
            error: state.error.clone(),
        }
    }

    pub fn request_activation(&self) {
        if let Ok(mut state) = self.state.lock() {
            state.force_activation = true;
            state.error = None;
        }
    }
}

fn receiver_loop(state: &Arc<Mutex<RadioState>>) -> std::io::Result<()> {
    let socket = UdpSocket::bind(("0.0.0.0", JOYSTICK_PORT))?;
    socket.set_read_timeout(Some(RECEIVE_TIMEOUT))?;

    let mut active_tx = None;
    let mut last_activation = Instant::now() - REACTIVATE_AFTER;
    let mut packet = [0_u8; 128];

    loop {
        let forced = state
            .lock()
            .map(|mut state| {
                let forced = state.force_activation;
                state.force_activation = false;
                forced
            })
            .unwrap_or(false);
        let stale = state
            .lock()
            .ok()
            .and_then(|state| state.last_packet)
            .is_none_or(|seen| seen.elapsed() >= REACTIVATE_AFTER);

        if (forced || stale) && last_activation.elapsed() >= REACTIVATE_AFTER {
            match active_tx.or_else(resolve_elrs_tx) {
                Some(ip) => match activate(ip) {
                    Ok(()) => {
                        active_tx = Some(ip);
                        if let Ok(mut state) = state.lock() {
                            state.transmitter = Some(ip);
                            state.error = None;
                        }
                    }
                    Err(error) => set_error(state, format!("Radio {ip}: {error}")),
                },
                None => set_error(state, "Nie znaleziono nadajnika elrs_tx.local".to_owned()),
            }
            last_activation = Instant::now();
        }

        let (length, sender) = match socket.recv_from(&mut packet) {
            Ok(received) => received,
            Err(error)
                if matches!(
                    error.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) =>
            {
                continue;
            }
            Err(error) => return Err(error),
        };
        let bytes = &packet[..length];

        if advertisement_name(bytes).is_some() {
            if last_activation.elapsed() >= REACTIVATE_AFTER {
                match activate(sender.ip()) {
                    Ok(()) => {
                        active_tx = Some(sender.ip());
                        if let Ok(mut state) = state.lock() {
                            state.transmitter = active_tx;
                            state.error = None;
                        }
                    }
                    Err(error) => set_error(state, format!("Radio {}: {error}", sender.ip())),
                }
                last_activation = Instant::now();
            }
            continue;
        }

        let Ok(channels) = ElrsChannels::parse(bytes) else {
            continue;
        };
        if let Some(expected) = active_tx {
            if expected != sender.ip() {
                continue;
            }
        } else {
            active_tx = Some(sender.ip());
        }

        if let Ok(mut state) = state.lock() {
            state.transmitter = active_tx;
            state.channel_count = channels.len();
            state.channels.fill(0);
            for index in 0..channels.len() {
                state.channels[index] = channels.raw(index).unwrap_or(0);
            }
            state.packet_count = state.packet_count.saturating_add(1);
            state.last_packet = Some(Instant::now());
            state.error = None;
        }
    }
}

fn resolve_elrs_tx() -> Option<IpAddr> {
    ("elrs_tx.local", 80)
        .to_socket_addrs()
        .ok()?
        .find(|address| address.is_ipv4())
        .map(|address| address.ip())
}

fn activate(ip: IpAddr) -> std::io::Result<()> {
    // ExpressLRS ignores StartSending while a previous joystick session is
    // active. This matters after an ESP OTA/restart and when increasing the
    // requested channel count, so explicitly replace the old session.
    let _ = send_control(ip, "action=joystick_end");
    let body =
        format!("action=joystick_begin&interval={UPDATE_INTERVAL_US}&channels={CHANNEL_COUNT}");
    send_control(ip, &body)
}

fn send_control(ip: IpAddr, body: &str) -> std::io::Result<()> {
    let address = SocketAddr::new(ip, 80);
    let mut stream = TcpStream::connect_timeout(&address, Duration::from_secs(2))?;
    stream.set_read_timeout(Some(Duration::from_secs(2)))?;
    stream.set_write_timeout(Some(Duration::from_secs(2)))?;
    let request = format!(
        "POST /udpcontrol HTTP/1.1\r\nHost: {ip}\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
        body.len()
    );
    stream.write_all(request.as_bytes())?;

    let mut response = String::new();
    stream.read_to_string(&mut response)?;
    let status = response.lines().next().unwrap_or_default();
    if !status.contains(" 200 ") {
        return Err(std::io::Error::other(format!(
            "aktywacja odrzucona ({status})"
        )));
    }
    Ok(())
}

fn advertisement_name(packet: &[u8]) -> Option<&str> {
    if packet.len() < 8 || &packet[..4] != b"ELRS" {
        return None;
    }
    let advertised_port = u16::from_le_bytes([packet[5], packet[6]]);
    let name_length = packet[7] as usize;
    if advertised_port != JOYSTICK_PORT || packet.len() != 8 + name_length {
        return None;
    }
    std::str::from_utf8(&packet[8..]).ok()
}

fn set_error(state: &Arc<Mutex<RadioState>>, error: String) {
    log::warn!("ELRS: {error}");
    if let Ok(mut state) = state.lock() {
        state.error = Some(error);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_elrs_advertisement() {
        let packet = [
            b'E', b'L', b'R', b'S', 1, 0xf8, 0x2a, 6, b'P', b'o', b'c', b'k', b'e', b't',
        ];
        assert_eq!(advertisement_name(&packet), Some("Pocket"));
    }
}
