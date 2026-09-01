use std::{
    env,
    io::{self, Read, Write},
    net::{IpAddr, SocketAddr, TcpStream, ToSocketAddrs, UdpSocket},
    time::{Duration, Instant},
};

use wobot_core::ElrsChannels;

const JOYSTICK_PORT: u16 = 11_000;
const UPDATE_INTERVAL_US: u32 = 10_000;
const CHANNEL_COUNT: u8 = 8;
const CHANGE_THRESHOLD: u16 = 250;

fn main() -> io::Result<()> {
    let explicit_tx = env::args().nth(1).map(|value| {
        value.parse::<IpAddr>().unwrap_or_else(|error| {
            eprintln!("invalid transmitter IP '{value}': {error}");
            std::process::exit(2);
        })
    });

    let socket = UdpSocket::bind(("0.0.0.0", JOYSTICK_PORT))?;
    socket.set_read_timeout(Some(Duration::from_secs(2)))?;
    println!("listening for ExpressLRS on UDP/{JOYSTICK_PORT}");

    let mut active_tx = None;
    let initial_tx = explicit_tx.or_else(resolve_elrs_tx);
    if let Some(ip) = initial_tx {
        activate(ip)?;
        active_tx = Some(ip);
    } else {
        println!("waiting for an ELRS broadcast; an IP may also be passed as the first argument");
    }

    let mut packet = [0_u8; 128];
    let mut last_print = Instant::now() - Duration::from_secs(1);
    let mut last_heartbeat = Instant::now() - Duration::from_secs(5);
    let mut last_reported = [0_u16; ElrsChannels::MAX_CHANNELS];
    let mut have_report = false;
    loop {
        let (length, sender) = match socket.recv_from(&mut packet) {
            Ok(received) => received,
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                if let Some(ip) = active_tx {
                    eprintln!("no channel packets; reactivating {ip}");
                    activate(ip)?;
                }
                continue;
            }
            Err(error) => return Err(error),
        };
        let bytes = &packet[..length];

        if let Some(name) = advertisement_name(bytes) {
            println!("found ELRS transmitter '{}' at {}", name, sender.ip());
            activate(sender.ip())?;
            active_tx = Some(sender.ip());
            continue;
        }

        if active_tx != Some(sender.ip()) {
            continue;
        }
        let Ok(channels) = ElrsChannels::parse(bytes) else {
            continue;
        };

        let mut values = [0_u16; ElrsChannels::MAX_CHANNELS];
        for (index, value) in values.iter_mut().enumerate().take(channels.len()) {
            *value = channels.raw(index).expect("validated channel index");
        }
        let changed = !have_report
            || values[..channels.len()]
                .iter()
                .zip(&last_reported)
                .any(|(current, previous)| current.abs_diff(*previous) >= CHANGE_THRESHOLD);
        let heartbeat = last_heartbeat.elapsed() >= Duration::from_secs(5);

        if (changed && last_print.elapsed() >= Duration::from_millis(100)) || heartbeat {
            print!("channels:");
            for (index, value) in values.iter().enumerate().take(channels.len()) {
                print!(" ch{}={value:>5}", index + 1);
            }
            println!();
            last_reported = values;
            have_report = true;
            last_print = Instant::now();
            if heartbeat {
                last_heartbeat = Instant::now();
            }
        }
    }
}

fn resolve_elrs_tx() -> Option<IpAddr> {
    let address = ("elrs_tx.local", 80)
        .to_socket_addrs()
        .ok()?
        .find(|address| address.is_ipv4())?;
    println!("resolved elrs_tx.local to {}", address.ip());
    Some(address.ip())
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

fn activate(ip: IpAddr) -> io::Result<()> {
    let address = SocketAddr::new(ip, 80);
    let mut stream = TcpStream::connect_timeout(&address, Duration::from_secs(2))?;
    stream.set_read_timeout(Some(Duration::from_secs(2)))?;
    let body =
        format!("action=joystick_begin&interval={UPDATE_INTERVAL_US}&channels={CHANNEL_COUNT}");
    let request = format!(
        "POST /udpcontrol HTTP/1.1\r\nHost: {ip}\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
        body.len()
    );
    stream.write_all(request.as_bytes())?;

    let mut response = String::new();
    stream.read_to_string(&mut response)?;
    let status = response.lines().next().unwrap_or_default();
    if !status.contains(" 200 ") {
        return Err(io::Error::other(format!(
            "ELRS activation failed: {status}"
        )));
    }
    println!("activated Wi-Fi Joystick at {ip} (100 Hz, {CHANNEL_COUNT} channels)");
    Ok(())
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

    #[test]
    fn rejects_advertisement_for_another_port() {
        let packet = [b'E', b'L', b'R', b'S', 1, 1, 2, 0];
        assert_eq!(advertisement_name(&packet), None);
    }
}
