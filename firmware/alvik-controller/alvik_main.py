"""Run manually with `import alvik_main; alvik_main.run()` on MicroPython."""

import json
import time

from control import Controller
from joystick import Joystick


def run(config_path="config.json"):
    import network

    with open(config_path) as source:
        config = json.load(source)
    controller = Controller(config, time.ticks_diff)
    if not isinstance(config["ssid"], str) or not config["ssid"]:
        raise ValueError("Set ssid in config.json")
    if not isinstance(config["password"], str):
        raise ValueError("password must be a string")
    if not config["dry_run"] and not config["calibration_confirmed"]:
        raise ValueError("Confirm channel calibration before enabling motors")

    robot = None
    radio = None

    def stop(reason):
        controller.last_packet = None
        controller.reset(reason)
        if robot is not None:
            robot.set_wheels_speed(0, 0)

    try:
        # Diagnostic mode does not initialize the carrier, UART or motors.
        if not config["dry_run"]:
            from arduino_alvik import ArduinoAlvik

            robot = ArduinoAlvik()
            robot.begin()
            robot.set_wheels_speed(0, 0)
        radio = Joystick(config["transmitter_ip"])
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        last_attempt = None
        last_report = None
        activated = False
        was_connected = False
        print("Wobot Alvik:", "DIAGNOSTIC (no motor output)" if config["dry_run"] else "MOTOR MODE")

        while True:
            now = time.ticks_ms()
            # Expire output before reading new packets or starting HTTP.
            output = controller.output(now)
            if robot is not None:
                robot.set_wheels_speed(*output)
            if not wlan.isconnected():
                stop("wifi_disconnected")
                activated = False
                was_connected = False
                if last_attempt is None or time.ticks_diff(now, last_attempt) >= 5000:
                    try:
                        wlan.connect(config["ssid"], config["password"])
                    except OSError:
                        print("Wi-Fi connection attempt failed")
                    last_attempt = now
                time.sleep_ms(20)
                continue
            if not was_connected:
                stop("wifi_connected")
                radio.reopen()
                last_attempt = None
                was_connected = True
                print("Alvik Wi-Fi IP:", wlan.ifconfig()[0])
            try:
                packet = radio.poll()
                if packet is not None and activated:
                    controller.receive(packet, time.ticks_ms())
                stale = (controller.last_packet is None or
                         time.ticks_diff(time.ticks_ms(), controller.last_packet) >= config["timeout_ms"])
                if (radio.transmitter is not None and stale and
                        (last_attempt is None or time.ticks_diff(now, last_attempt) >= 2000)):
                    stop("activating_radio")
                    activated = False
                    print("Activating ELRS Wi-Fi Joystick:", radio.transmitter)
                    radio.activate()
                    activated = True
                    last_attempt = time.ticks_ms()
                output = controller.output(time.ticks_ms())
                if robot is not None:
                    robot.set_wheels_speed(*output)
            except OSError:
                stop("network_error")
                activated = False
                last_attempt = time.ticks_ms()
                radio.reopen()
                print("Radio network error; motors stopped, retry in 2 s")
            if last_report is None or time.ticks_diff(now, last_report) >= 500:
                print("channels:", controller.channels, "profile:", controller.profile,
                      "state:", controller.reason, "requested rpm:", controller.wheels,
                      "output rpm:", controller.output(time.ticks_ms()))
                last_report = now
            time.sleep_ms(20)
    finally:
        # Ctrl-C and unhandled errors stop the carrier before releasing sockets.
        try:
            if robot is not None:
                robot.stop()
        finally:
            if radio is not None:
                radio.close()
