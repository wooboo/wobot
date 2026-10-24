import json
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

FIRMWARE = Path(__file__).resolve().parents[1] / "firmware" / "alvik-controller"
sys.path.insert(0, str(FIRMWARE))
from control import Controller, advertisement, parse_channels, validate
from joystick import Joystick, control_request, send_control
import alvik_main


def config():
    value = json.loads((FIRMWARE / "config.example.json").read_text())
    value.update(dry_run=False, calibration_confirmed=True)
    return value


def packet(arm=0, throttle=16384, steering=16384, strafe=16384,
           profile=32767, pedal=0, direction=32767):
    values = [steering, throttle, pedal, strafe, arm, direction, 16384, profile, 16384, 16384]
    return bytes([1, len(values)]) + struct.pack("<10H", *values)


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.drive = Controller(config(), lambda a, b: a - b)

    def arm(self):
        self.drive.receive(packet(), 0)
        self.drive.receive(packet(arm=32767), 10)
        self.assertTrue(self.drive.armed)

    def test_wire_format_matches_rust_decoder(self):
        self.assertEqual(parse_channels(bytes([1, 2, 0, 0, 255, 127])), (0, 32767))
        for data in (b"", b"\x01", b"\x02\x01\x00\x00", b"\x01\x00",
                     b"\x01\x11" + b"\0" * 34, b"\x01\x01\xff\xff",
                     packet() + b"\0", packet()[:-1]):
            with self.subTest(data=data), self.assertRaises(ValueError):
                parse_channels(data)

    def test_startup_requires_disarm(self):
        self.drive.receive(packet(arm=32767, throttle=32767), 0)
        self.assertEqual(self.drive.output(0), (0, 0))
        self.assertEqual(self.drive.reason, "awaiting_disarm")

    def test_neutral_required_and_unsafe_arm_requires_new_disarm(self):
        self.drive.receive(packet(), 0)
        self.drive.receive(packet(arm=32767, steering=32767), 10)
        self.assertEqual(self.drive.reason, "controls_not_idle")
        self.drive.receive(packet(arm=32767), 20)
        self.assertFalse(self.drive.armed)

    def test_forward_reverse_and_limit(self):
        self.arm()
        self.drive.receive(packet(arm=32767, throttle=32767), 20)
        self.assertEqual(self.drive.output(20), (20, 20))
        self.drive.receive(packet(arm=32767, throttle=0), 30)
        self.assertEqual(self.drive.output(30), (-20, -20))

    def test_steering_reduces_inner_wheel_without_pivot(self):
        self.arm()
        self.drive.receive(packet(arm=32767, throttle=32767, steering=32767), 20)
        self.assertEqual(self.drive.output(20), (20, 6))
        self.drive.receive(packet(arm=32767, steering=32767), 30)
        self.assertEqual(self.drive.output(30), (0, 0))

    def test_disarm_and_switch_middle_stop_immediately(self):
        self.arm()
        self.drive.receive(packet(arm=32767, throttle=32767), 20)
        self.drive.receive(packet(arm=16384, throttle=32767), 21)
        self.assertEqual(self.drive.output(21), (0, 0))
        self.assertFalse(self.drive.armed)

    def test_timeout_boundary_and_return_of_signal(self):
        self.arm()
        self.drive.receive(packet(arm=32767, throttle=32767), 20)
        self.assertEqual(self.drive.output(269), (20, 20))
        self.assertEqual(self.drive.output(270), (0, 0))
        self.drive.receive(packet(arm=32767), 271)
        self.assertFalse(self.drive.armed)

    def test_packet_after_loop_stall_cannot_revive_arm(self):
        self.arm()
        self.drive.receive(packet(arm=32767, throttle=32767), 1000)
        self.assertEqual(self.drive.output(1000), (0, 0))

    def test_micropython_tick_wrap(self):
        self.drive.diff = lambda a, b: ((a - b + 512) % 1024) - 512
        self.drive.receive(packet(), 1000)
        self.drive.receive(packet(arm=32767), 1010)
        self.drive.receive(packet(arm=32767, throttle=32767), 1020)
        self.assertEqual(self.drive.output(10), (20, 20))
        self.assertEqual(self.drive.output(246), (0, 0))

    def test_bad_frame_invalidates_arm(self):
        self.arm()
        self.drive.receive(packet()[:-1], 20)
        self.drive.receive(packet(arm=32767), 21)
        self.assertFalse(self.drive.armed)

    def test_missing_required_channel(self):
        self.arm()
        self.drive.receive(b"\x01\x01\0\0", 20)
        self.assertFalse(self.drive.armed)

    def test_strafe_stops_and_requires_rearm(self):
        self.arm()
        self.drive.receive(packet(arm=32767, throttle=32767, strafe=32767), 20)
        self.assertEqual(self.drive.output(20), (0, 0))
        self.drive.receive(packet(arm=32767), 21)
        self.assertFalse(self.drive.armed)

    def test_profile_change_requires_rearm_and_pedal_direction_works(self):
        self.arm()
        self.drive.receive(packet(arm=32767, profile=0), 20)
        self.assertFalse(self.drive.armed)
        self.drive.receive(packet(profile=0), 21)
        self.drive.receive(packet(arm=32767, profile=0), 22)
        self.drive.receive(packet(arm=32767, profile=0, pedal=32767, direction=0), 23)
        self.assertEqual(self.drive.output(23), (-20, -20))
        self.drive.receive(packet(arm=32767, profile=0, pedal=32767, direction=16384), 24)
        self.assertEqual(self.drive.output(24), (0, 0))

    def test_diagnostic_and_uncalibrated_never_output_motion(self):
        for key, value in (("dry_run", True), ("calibration_confirmed", False)):
            self.setUp()
            self.drive.config[key] = value
            self.arm()
            self.drive.receive(packet(arm=32767, throttle=32767), 20)
            self.assertEqual(self.drive.output(20), (0, 0))
            self.assertEqual(self.drive.wheels, (20, 20))

    def test_asymmetric_calibration_and_inversion(self):
        self.drive.config["ranges"][1] = [1000, 10000, 30000]
        self.drive.config["inverted"] = ["throttle"]
        self.drive.receive(packet(throttle=10000), 0)
        self.assertEqual(self.drive.axis("throttle"), 0)
        self.drive.receive(packet(throttle=1000), 1)
        self.assertEqual(self.drive.axis("throttle"), 1000)
        self.drive.receive(packet(throttle=30000), 2)
        self.assertEqual(self.drive.axis("throttle"), -1000)

    def test_reject_unsafe_config(self):
        for key, value in (("dry_run", "false"), ("max_rpm", 1000),
                           ("timeout_ms", 0), ("switch_deadband", 500),
                           ("inverted", ["typo"]), ("ranges", [[0, 0, 0]] * 10)):
            candidate = config()
            candidate[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate(candidate)
        candidate = config()
        candidate["channels"]["arm"] = 1
        with self.assertRaises(ValueError):
            validate(candidate)


class FakeSocket:
    def __init__(self, packets=(), response=b"HTTP/1.1 200 OK\r\n\r\n"):
        self.packets = list(packets)
        self.response = response
        self.closed = False
        self.sent = None

    def bind(self, address):
        self.bound = address

    def setblocking(self, value):
        pass

    def settimeout(self, value):
        pass

    def connect(self, address):
        pass

    def sendall(self, data):
        self.sent = data

    def recv(self, count):
        data, self.response = self.response[:count], self.response[count:]
        return data

    def recvfrom(self, count):
        if not self.packets:
            raise OSError(11)
        return self.packets.pop(0)

    def close(self):
        self.closed = True


class TransportTests(unittest.TestCase):
    def test_http_wire_compatibility(self):
        request = control_request("192.0.2.1", "joystick_begin")
        header, body = request.split(b"\r\n\r\n")
        self.assertTrue(header.startswith(b"POST /udpcontrol HTTP/1.1\r\n"))
        self.assertIn(("Content-Length: " + str(len(body))).encode(), header)
        self.assertEqual(body, b"action=joystick_begin&interval=10000&channels=10")

    def test_advertisement_validation(self):
        self.assertEqual(advertisement(b"ELRS\x01\xf8\x2a\x06Pocket"), "Pocket")
        for data in (b"ELRS", b"ELRS\x01\x01\x02\x01x", b"ELRS\x01\xf8\x2a\x01\xff"):
            self.assertIsNone(advertisement(data))

    def test_discover_then_filter_sender_and_keep_latest(self):
        source = ("192.0.2.1", 11000)
        fake = FakeSocket([(b"ELRS\x01\xf8\x2a\x06Pocket", source),
                           (packet(), source), (packet(arm=32767), source),
                           (packet(throttle=0), ("192.0.2.2", 11000))])
        with patch("joystick.socket.socket", return_value=fake):
            radio = Joystick()
            self.assertEqual(radio.poll(), packet(arm=32767))
            self.assertEqual(radio.transmitter, source[0])

    def test_pinned_ip_cannot_be_changed_by_advertisement(self):
        fake = FakeSocket([(b"ELRS\x01\xf8\x2a\x06Pocket", ("192.0.2.2", 11000))])
        with patch("joystick.socket.socket", return_value=fake):
            radio = Joystick("192.0.2.1")
            self.assertIsNone(radio.poll())
            self.assertEqual(radio.transmitter, "192.0.2.1")

    def test_backlog_is_rejected(self):
        fake = FakeSocket([(packet(), ("192.0.2.1", 11000))] * 32)
        with patch("joystick.socket.socket", return_value=fake):
            radio = Joystick("192.0.2.1")
            with self.assertRaises(OSError):
                radio.poll()

    def test_activation_ends_old_session_and_discards_queue(self):
        old, new = FakeSocket(), FakeSocket()
        with patch("joystick.socket.socket", side_effect=[old, new]), patch("joystick.send_control") as send:
            radio = Joystick("192.0.2.1")
            radio.activate()
            self.assertEqual([call.args for call in send.call_args_list],
                             [("192.0.2.1", "joystick_end"), ("192.0.2.1", "joystick_begin")])
            self.assertTrue(old.closed)
            self.assertIs(radio.socket, new)

    def test_rejected_http_always_closes_socket(self):
        fake = FakeSocket(response=b"HTTP/1.1 500 Error\r\n\r\n")
        with patch("joystick.socket.socket", return_value=fake), self.assertRaises(OSError):
            send_control("192.0.2.1", "joystick_begin")
        self.assertTrue(fake.closed)


class RuntimeTests(unittest.TestCase):
    def exercise(self, dry_run, fail_poll=False, disconnect_while_moving=False):
        import types
        settings = config()
        settings.update(dry_run=dry_run, ssid="test")
        events = []
        elapsed = [0]

        class Robot:
            def begin(self):
                events.append("begin")

            def set_wheels_speed(self, left, right):
                events.append((left, right))

            def stop(self):
                events.append("stop")

        class Radio:
            transmitter = "192.0.2.1"

            def __init__(self, ip):
                pass

            def reopen(self):
                events.append("reopen")

            def poll(self):
                if fail_poll:
                    raise OSError("network failure")
                if disconnect_while_moving:
                    return {20: packet(), 40: packet(arm=32767),
                            60: packet(arm=32767, throttle=32767)}.get(elapsed[0])
                return None

            def activate(self):
                events.append("activate")

            def close(self):
                events.append("close")

        def sleep(delay):
            elapsed[0] += delay
            if not disconnect_while_moving or elapsed[0] >= 100:
                raise KeyboardInterrupt()

        wlan = types.SimpleNamespace(active=lambda value: None,
                                     isconnected=lambda: not disconnect_while_moving or elapsed[0] < 80,
                                     ifconfig=lambda: ("192.0.2.10",))
        modules = {"network": types.SimpleNamespace(STA_IF=0, WLAN=lambda mode: wlan),
                   "arduino_alvik": types.SimpleNamespace(ArduinoAlvik=Robot)}
        clock = types.SimpleNamespace(ticks_ms=lambda: elapsed[0], ticks_diff=lambda a, b: a-b,
                                      sleep_ms=sleep)
        with patch.dict(sys.modules, modules), patch("alvik_main.time", clock), \
                patch("alvik_main.Joystick", Radio), patch("alvik_main.json.load", return_value=settings):
            with self.assertRaises(KeyboardInterrupt):
                alvik_main.run(str(FIRMWARE / "config.example.json"))
        return events

    def test_diagnostic_mode_does_not_initialize_robot(self):
        events = self.exercise(True)
        self.assertNotIn("begin", events)
        self.assertNotIn("stop", events)
        self.assertNotIn((0, 0), events)
        self.assertIn("activate", events)
        self.assertEqual(events[-1], "close")

    def test_motors_stop_before_blocking_activation_and_on_ctrl_c(self):
        events = self.exercise(False)
        self.assertEqual(events[events.index("activate") - 1], (0, 0))
        self.assertEqual(events[-2:], ["stop", "close"])

    def test_network_error_stops_before_reopening_socket(self):
        events = self.exercise(False, fail_poll=True)
        self.assertEqual(events[-4:], [(0, 0), "reopen", "stop", "close"])

    def test_wifi_disconnect_while_driving_commands_zero(self):
        events = self.exercise(False, disconnect_while_moving=True)
        self.assertIn((20, 20), events)
        self.assertEqual(events[-3:], [(0, 0), "stop", "close"])


if __name__ == "__main__":
    unittest.main()
