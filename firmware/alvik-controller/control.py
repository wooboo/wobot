"""Portable ELRS Wi-Fi decoder and Alvik drive state (also tested on CPython)."""


def parse_channels(packet):
    if len(packet) < 2 or packet[0] != 1:
        raise ValueError("not an ELRS channel frame")
    count = packet[1]
    if not 1 <= count <= 16 or len(packet) != 2 + count * 2:
        raise ValueError("invalid ELRS frame length")
    values = tuple(packet[i] | packet[i + 1] << 8 for i in range(2, len(packet), 2))
    if any(value > 32767 for value in values):
        raise ValueError("invalid ELRS channel value")
    return values


def advertisement(packet):
    if (len(packet) < 8 or packet[:4] != b"ELRS"
            or packet[5] | packet[6] << 8 != 11000
            or len(packet) != 8 + packet[7]):
        return None
    try:
        return packet[8:].decode("utf-8")
    except UnicodeError:
        return None


def validate(config):
    for key in ("dry_run", "calibration_confirmed"):
        if type(config[key]) is not bool:
            raise ValueError(key + " must be boolean")
    for key, low, high in (("max_rpm", 1, 60), ("timeout_ms", 50, 1000),
                           ("deadband", 0, 300), ("switch_deadband", 0, 450)):
        if type(config[key]) is not int or not low <= config[key] <= high:
            raise ValueError("invalid " + key)
    names = ("steering", "throttle", "pedal", "strafe", "arm", "direction", "profile")
    used = []
    for name in names:
        channel = config["channels"][name]
        if type(channel) is not int or not 1 <= channel <= 10 or channel in used:
            raise ValueError("invalid or duplicate channel: " + name)
        used.append(channel)
    if len(config["ranges"]) != 10:
        raise ValueError("10 calibration ranges required")
    for triple in config["ranges"]:
        if len(triple) != 3 or any(type(value) is not int for value in triple):
            raise ValueError("invalid calibration range")
        low, center, high = triple
        if not 0 <= low < center < high <= 32767 or high - low < 500:
            raise ValueError("invalid calibration range")
    if any(name not in names for name in config["inverted"]):
        raise ValueError("unknown inverted function")


class Controller:
    def __init__(self, config, ticks_diff):
        validate(config)
        self.config = config
        self.diff = ticks_diff
        self.profile = None
        self.channels = None
        self.last_packet = None
        self.reset("awaiting_disarm")

    def reset(self, reason):
        self.saw_disarm = False
        self.armed = False
        self.wheels = (0, 0)
        self.reason = reason

    def _raw(self, name):
        index = self.config["channels"][name] - 1
        low, center, high = self.config["ranges"][index]
        raw = min(high, max(low, self.channels[index]))
        return raw, low, center, high

    def axis(self, name, pedal=False):
        raw, low, center, high = self._raw(name)
        if pedal:
            value = (raw - low) * 1000 // (high - low)
            if name in self.config["inverted"]:
                value = 1000 - value
        else:
            value = ((raw - center) * 1000 // (high - center) if raw >= center
                     else -((center - raw) * 1000 // (center - low)))
            if name in self.config["inverted"]:
                value = -value
        deadband = self.config["deadband"]
        magnitude = abs(value)
        if magnitude <= deadband:
            return 0
        result = (magnitude - deadband) * 1000 // (1000 - deadband)
        return result if value >= 0 else -result

    def switch(self, name):
        raw, low, _, high = self._raw(name)
        middle = low + (high - low) // 2
        margin = (high - low) * self.config["switch_deadband"] // 1000
        value = 1 if raw > middle + margin else -1 if raw < middle - margin else 0
        return -value if name in self.config["inverted"] else value

    def receive(self, packet, now):
        # A new packet after a stall must not revive the previous ARM latch.
        self.output(now)
        try:
            channels = parse_channels(packet)
            if len(channels) < max(self.config["channels"].values()):
                raise ValueError("required channel missing")
        except ValueError:
            self.last_packet = None
            self.reset("invalid_packet")
            return
        self.channels = channels
        self.last_packet = now
        profile = "stick" if self.switch("profile") == 1 else "pedal"
        if self.profile != profile:
            self.profile = profile
            self.reset("profile_changed")
        if self.switch("arm") != 1:
            self.reset("disarmed")
            self.saw_disarm = True
            return
        if not self.saw_disarm:
            self.reset("awaiting_disarm")
            return
        throttle = self.axis("throttle") if profile == "stick" else self.axis("pedal", True)
        steering = self.axis("steering")
        strafe = self.axis("strafe")
        if not self.armed and (throttle != 0 or steering != 0 or strafe != 0):
            self.reset("controls_not_idle")
            return
        if strafe != 0:
            self.reset("strafe_unsupported")
            return
        self.armed = True
        if profile == "pedal":
            direction = self.switch("direction")
            throttle *= direction
            if direction == 0:
                self.wheels = (0, 0)
                self.reason = "neutral"
                return
        # Match Wobot's car-like steering: no pivot on steering alone.
        inner = throttle * (1000 - abs(steering) * 700 // 1000) / 1000
        left, right = (throttle, inner) if steering >= 0 else (inner, throttle)
        scale = self.config["max_rpm"] / 1000
        self.wheels = (left * scale, right * scale)
        self.reason = "armed"

    def output(self, now):
        if self.last_packet is None or self.diff(now, self.last_packet) >= self.config["timeout_ms"]:
            self.reset("radio_timeout")
        if self.config["dry_run"] or not self.config["calibration_confirmed"]:
            return (0, 0)
        return self.wheels
