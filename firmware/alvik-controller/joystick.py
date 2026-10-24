"""Same /udpcontrol + UDP/11000 transport as Wobot's Rust radio service."""

import socket

from control import advertisement

PORT = 11000


def ipv4(value):
    parts = value.split(".")
    return len(parts) == 4 and all(part.isdigit() and 0 <= int(part) <= 255 for part in parts)


def control_request(ip, action):
    body = "action=" + action
    if action == "joystick_begin":
        body += "&interval=10000&channels=10"
    return ("POST /udpcontrol HTTP/1.1\r\nHost: " + ip
            + "\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: "
            + str(len(body)) + "\r\nConnection: close\r\n\r\n" + body).encode()


def send_control(ip, action):
    stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        stream.settimeout(0.5)
        stream.connect((ip, 80))
        stream.sendall(control_request(ip, action))
        status = b""
        while b"\r\n" not in status and len(status) < 256:
            chunk = stream.recv(64)
            if not chunk:
                break
            status += chunk
        fields = status.split(b"\r\n", 1)[0].split()
        if len(fields) < 2 or fields[1] != b"200":
            raise OSError("ELRS activation rejected")
    finally:
        stream.close()


class Joystick:
    def __init__(self, transmitter_ip=""):
        if transmitter_ip and not ipv4(transmitter_ip):
            raise ValueError("transmitter_ip must be empty or an IPv4 address")
        self.transmitter = transmitter_ip or None
        self.socket = None
        self.reopen()

    def reopen(self):
        self.close()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", PORT))
        self.socket.setblocking(False)

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None

    def activate(self):
        # Caller must stop the motors before any potentially blocking HTTP.
        try:
            send_control(self.transmitter, "joystick_end")
        except OSError:
            pass
        send_control(self.transmitter, "joystick_begin")
        # Discard packets queued while HTTP was blocking, including old ARM.
        self.reopen()

    def poll(self):
        latest = None
        for _ in range(32):
            try:
                packet, sender = self.socket.recvfrom(128)
            except OSError as error:
                if error.args and error.args[0] in (11, 35):  # EAGAIN / EWOULDBLOCK
                    return latest
                raise
            name = advertisement(packet)
            if name is not None:
                if self.transmitter is None:
                    self.transmitter = sender[0]
                    print("ELRS discovered:", name, self.transmitter)
                continue
            if self.transmitter == sender[0]:
                latest = packet
        # Never drive using a backlog whose reception times are unknown.
        raise OSError("UDP backlog; discarding pending commands")
