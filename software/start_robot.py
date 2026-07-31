#!/usr/bin/env python3
"""Waits for a PS4 controller button press, then launches the robot"""

import subprocess
import sys
import time
import os

try:
    import evdev
    from evdev import ecodes
except ImportError:
    print("Installing python3-evdev...")
    subprocess.run([sys.executable, "-m", "pip", "install", "evdev"])
    import evdev
    from evdev import ecodes

COMPOSE_DIR = os.path.dirname(os.path.abspath(__file__))
CONTAINER = "dingoquadruped-master-dingoquadruped-1"
LAUNCH_CMD = [
    "docker", "compose", "-f", os.path.join(COMPOSE_DIR, "docker-compose.yaml"),
    "exec", "-T", "dingoquadruped",
    "bash", "-c",
    "source /opt/ros/noetic/setup.bash && source /dingo_ws/devel/setup.bash && roslaunch --screen dingo dingo.launch use_joystick:=1"
]
STOP_CMD = [
    "docker", "compose", "-f", os.path.join(COMPOSE_DIR, "docker-compose.yaml"),
    "exec", "-T", "dingoquadruped",
    "bash", "-c", "pkill -INT -f roslaunch"
]

LED_READY = (160, 0, 255)     # purple: launcher up, waiting for PS

_SET_LED_SNIPPET = """
led=$(ls -d /sys/class/leds/*:rgb:indicator 2>/dev/null | head -1)
if [ -n "$led" ]; then
    echo "%(r)d %(g)d %(b)d" > "$led/multi_intensity"
    echo 255 > "$led/brightness"
else
    base=$(ls -d /sys/class/leds/*:red 2>/dev/null | head -1 | sed 's/:red$//')
    if [ -n "$base" ]; then
        [ -f "$base:global/brightness" ] && echo 1 > "$base:global/brightness"
        echo %(r)d > "$base:red/brightness"
        echo %(g)d > "$base:green/brightness"
        echo %(b)d > "$base:blue/brightness"
    fi
fi
"""


def set_led(color):
    """Set the light bar. Never fatal -- a missing container or an unbound
    light bar must not stop the robot being launched."""
    r, g, b = color
    try:
        subprocess.run(
            ["docker", "compose", "-f",
             os.path.join(COMPOSE_DIR, "docker-compose.yaml"),
             "exec", "-T", "dingoquadruped",
             "bash", "-c", _SET_LED_SNIPPET % {"r": r, "g": g, "b": b}],
            cwd=COMPOSE_DIR, timeout=10)
    except Exception as e:
        print("(could not set light bar: %s)" % e)


PS_BUTTON = 316  # PS button event code


def find_controller():
    """Find the PS4 controller's main gamepad input device."""
    while True:
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for dev in devices:
            if "wireless controller" not in dev.name.lower():
                continue
            keys = dev.capabilities().get(ecodes.EV_KEY, [])
            if PS_BUTTON in keys:
                print(f"Found controller: {dev.name} at {dev.path}")
                return dev
        print("Waiting for PS4 controller...")
        time.sleep(2)


def wait_for_button(device, button_code):
    """Block until a specific button is pressed."""
    for event in device.read_loop():
        if event.type == ecodes.EV_KEY and event.code == button_code and event.value == 1:
            return


def main():
    print("=" * 50)
    print("DINGO ROBOT LAUNCHER")
    print("Press PS button to start the robot")
    print("Press PS button again to stop")
    print("=" * 50)

    set_led(LED_READY)   # purple: waiting for the first PS press

    while True:
        controller = find_controller()

        print("\n>> Press PS button to START the robot...")
        wait_for_button(controller, PS_BUTTON)
        print("Starting robot...")

        proc = subprocess.Popen(LAUNCH_CMD, cwd=COMPOSE_DIR)

        try:
            print(">> Robot running. Press PS button to STOP...")
            wait_for_button(controller, PS_BUTTON)
        except (OSError, IOError):
            print("Controller disconnected, stopping robot...")

        print("Stopping robot...")
        # Signal the remote roslaunch inside the container first (SIGINT,
        # same as Ctrl-C) so it shuts down its managed nodes cleanly.
        subprocess.run(STOP_CMD, cwd=COMPOSE_DIR)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            print("Remote roslaunch did not exit in time, forcing local client closed")
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

        set_led(LED_READY)   # purple again: ready for the next start
        print("Robot stopped.\n")
        time.sleep(1)


if __name__ == "__main__":
    main()
