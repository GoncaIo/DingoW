#!/usr/bin/env python3
"""Controls the DualShock 4 light bar via the kernel LED interface.

Two sysfs layouts exist depending on which kernel driver bound the pad:
  - hid-playstation (kernel >= 6.2): one multicolor LED,
      /sys/class/leds/<dev>:rgb:indicator/{multi_intensity,brightness}
  - hid-sony (older kernels): three separate LEDs,
      /sys/class/leds/<dev>:{red,green,blue}/brightness

Writes require root, which the Docker container runs as.
"""

import glob
import os
import rospy

from dingo_control.State import BehaviorState

STATE_COLORS = {
    BehaviorState.REST: (255, 255, 255),      # white
    BehaviorState.TROT: (255, 100, 0),         # orange
    BehaviorState.HOP: (0, 255, 0),          # green
    BehaviorState.FINISHHOP: (255, 100, 0),   # orange
    BehaviorState.DEACTIVATED: (255, 0, 0),   # red
    BehaviorState.WHEELED: (99, 134, 2),    # khaki green
    BehaviorState.CRAWL: (0, 255, 255),     # cyan
    BehaviorState.PAW: (255, 20, 147),      # hot pink
}
EXTERNAL_MODE_COLOR = (0, 0, 255)             # blue

# Hand-picked colors shown when IMU stabilization is off, keyed by the same
# BehaviorState as STATE_COLORS. Distinct hues (not just dimmer) so the
# difference is unmistakable even on a washed-out light bar.
DIMMED_STATE_COLORS = {
    BehaviorState.REST: (25, 35, 45),         # dim blue-gray
    BehaviorState.TROT: (60, 24, 0),          # dark burnt orange
    BehaviorState.HOP: (0, 40, 0),            # dark green
    BehaviorState.FINISHHOP: (60, 24, 0),     # dark burnt orange
    BehaviorState.DEACTIVATED: (50, 0, 0),    # dark red
    BehaviorState.WHEELED: (35, 45, 10),      # dark olive
    BehaviorState.CRAWL: (0, 45, 45),         # dark cyan
    BehaviorState.PAW: (45, 4, 26),           # dark pink
}


class DS4Led:
    def __init__(self):
        self.led_path = None
        self.mode = None
        self._last_color = None
        self._find_led()

    def _find_led(self):
        # hid-playstation: single multicolor LED
        paths = glob.glob("/sys/class/leds/*:rgb:indicator")
        if paths:
            self.led_path = paths[0]
            self.mode = "multicolor"
            rospy.loginfo("DS4 light bar found (multicolor): %s", self.led_path)
            return
        # hid-sony: separate red/green/blue LEDs sharing a prefix
        for red in glob.glob("/sys/class/leds/*:red"):
            base = red[: -len(":red")]
            if os.path.isdir(base + ":green") and os.path.isdir(base + ":blue"):
                self.led_path = base
                self.mode = "separate"
                rospy.loginfo("DS4 light bar found (separate RGB): %s", self.led_path)
                return
        rospy.logwarn("DS4 light bar not found in /sys/class/leds; LED feedback disabled")

    def set_color(self, color):
        """Set the light bar to an (r, g, b) tuple, 0-255 each."""
        if self.mode is None or color == self._last_color:
            return
        r, g, b = color
        try:
            if self.mode == "multicolor":
                with open(os.path.join(self.led_path, "multi_intensity"), "w") as f:
                    f.write("%d %d %d" % (r, g, b))
                with open(os.path.join(self.led_path, "brightness"), "w") as f:
                    f.write("255")
            else:
                # hid-sony has a master switch that must be on for the
                # color channels to show
                global_path = self.led_path + ":global/brightness"
                if os.path.exists(global_path):
                    with open(global_path, "w") as f:
                        f.write("1")
                for name, value in (("red", r), ("green", g), ("blue", b)):
                    with open("%s:%s/brightness" % (self.led_path, name), "w") as f:
                        f.write(str(value))
            self._last_color = color
        except Exception as e:
            rospy.logwarn_throttle(10, "Failed to set DS4 light bar: %s" % e)

    def show_state(self, behavior_state, dimmed=False):
        """Set the light bar color for the given BehaviorState.

        dimmed=True (IMU stabilization off) shows a hand-picked darker
        variant (DIMMED_STATE_COLORS) rather than a scaled-down version, so
        the difference is unmistakable rather than a subtle brightness shift.
        """
        if dimmed:
            color = DIMMED_STATE_COLORS.get(behavior_state, (25, 35, 45))
        else:
            color = STATE_COLORS.get(behavior_state, (255, 255, 255))
        self.set_color(color)

    def show_external_mode(self):
        """Set the light bar color for external command mode."""
        self.set_color(EXTERNAL_MODE_COLOR)
