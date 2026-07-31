#!/usr/bin/env python3
"""Wheel motor interface via Pi Pico W over UART."""

import serial
import threading
import math
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray


SERIAL_PORT = "/dev/ttyAMA2"
SERIAL_BAUD = 115200
MAX_RPM = 20.0

REVERSE_STEERING_LIKE_A_CAR = True
# Fraction of v_max below which the flip is suppressed, so the sign cannot
# chatter around a standstill and spin-on-the-spot keeps its absolute sense.
REVERSE_STEER_DEADBAND = 0.05 * (MAX_RPM / 60.0) * 2.0 * math.pi * 0.035

# Left-side motors (FL, BL) are mounted mirrored to the right side, so they
# need reversed sign for the same rolling direction. Order: [FR, FL, BR, BL].
MOTOR_DIRECTIONS = [1, -1, 1, -1]


class WheelInterface:
    def __init__(self, config, port=SERIAL_PORT, baudrate=SERIAL_BAUD):
        self.config = config
        # Measured distance between left and right wheel contact points in the
        # normal standing pose (2*delta_y underestimated it at 0.221 m).
        self.track_width = 0.265
        self.wheel_radius = config.wheel_radius

        self.targets = [0.0, 0.0, 0.0, 0.0]
        self.rpms = [0.0, 0.0, 0.0, 0.0]
        self.pwms = [0, 0, 0, 0]
        self.positions = [0, 0, 0, 0]
        self._lock = threading.Lock()

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
        except Exception as e:
            rospy.logerr("Failed to connect to Pico on {}: {}".format(port, e))
            raise

        self._stop_event = threading.Event()
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

        self.rpm_pub = rospy.Publisher("/wheel_rpms", Float64MultiArray, queue_size=10)
        self.pos_pub = rospy.Publisher("/wheel_positions", Float64MultiArray, queue_size=10)

        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("Pico wheel interface initialized on {}".format(port))

    # ── UART communication ───────────────────────────────────────────

    def _send(self, cmd):
        try:
            self.ser.write((cmd + "\n").encode())
        except Exception as e:
            rospy.logwarn("UART send error: {}".format(e))

    def _read_loop(self):
        buf = ""
        while not self._stop_event.is_set():
            try:
                raw = self.ser.read(self.ser.in_waiting or 1)
                if not raw:
                    continue
                buf += raw.decode(errors="replace")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line.startswith("D "):
                        self._parse_status(line[2:])
            except Exception:
                pass

    def _parse_status(self, data):
        try:
            parts = data.split(",")
            if len(parts) != 16:
                return
            with self._lock:
                for i in range(4):
                    self.targets[i] = float(parts[i * 4])
                    self.rpms[i] = float(parts[i * 4 + 1])
                    self.pwms[i] = int(parts[i * 4 + 2])
                    self.positions[i] = int(parts[i * 4 + 3])

            self.rpm_pub.publish(Float64MultiArray(data=list(self.rpms)))
            self.pos_pub.publish(Float64MultiArray(data=[float(p) for p in self.positions]))
        except (ValueError, IndexError):
            pass

    # ── Velocity to wheel RPMs ───────────────────────────────────────

    def _velocity_to_rpms(self, v_x, yaw_rate):
        """Convert body velocity command to individual wheel RPMs."""
        v_right = v_x + yaw_rate * self.track_width / 2.0
        v_left = v_x - yaw_rate * self.track_width / 2.0

        circumference = 2.0 * math.pi * self.wheel_radius
        rpm_right = (v_right / circumference) * 60.0
        rpm_left = (v_left / circumference) * 60.0

        return [
            np.clip(rpm_right, -MAX_RPM, MAX_RPM),  # FR
            np.clip(rpm_left, -MAX_RPM, MAX_RPM),    # FL
            np.clip(rpm_right, -MAX_RPM, MAX_RPM),   # BR
            np.clip(rpm_left, -MAX_RPM, MAX_RPM),    # BL
        ]

    # ── Public API ───────────────────────────────────────────────────

    def drive(self, command):
        """Drive wheels based on the analog trigger throttle from the controller."""
        v_max = (MAX_RPM / 60.0) * 2.0 * math.pi * self.wheel_radius
        v_x = command.wheel_throttle * v_max
        steer = command.wheel_yaw_rate / self.config.max_yaw_rate
        steer = max(-1.0, min(1.0, steer))

        if REVERSE_STEERING_LIKE_A_CAR and v_x < -REVERSE_STEER_DEADBAND:
            steer = -steer

        v_diff = 0.75 * v_max * steer
        yaw_rate = 2.0 * v_diff / self.track_width
        rpms = self._velocity_to_rpms(v_x, yaw_rate)
        self.set_motor_targets(rpms)

    def set_motor_targets(self, rpms):
        """Send individual RPM targets for all 4 wheel motors."""
        vals = [
            max(-MAX_RPM, min(MAX_RPM, float(r))) * d
            for r, d in zip(rpms, MOTOR_DIRECTIONS)
        ]
        self._send("T {:.1f},{:.1f},{:.1f},{:.1f}".format(*vals))

    def set_all_motor_targets(self, rpm):
        """Set all 4 wheel motors to the same forward RPM."""
        rpm = max(-MAX_RPM, min(MAX_RPM, rpm))
        self._send("T {:.1f},{:.1f},{:.1f},{:.1f}".format(
            *[rpm * d for d in MOTOR_DIRECTIONS]))

    def stop(self):
        """Stop all wheel motors."""
        self._send("S")

    def emergency_stop(self):
        """Emergency stop — disables all motor drivers on the Pico."""
        self._send("X")

    def enable(self):
        """Re-enable motor drivers after emergency stop."""
        self._send("G")

    def reset_encoders(self):
        """Reset all encoder position counters to zero."""
        self._send("R")

    def set_pid(self, kp, ki, kd):
        """Update PID gains on the Pico."""
        self._send("P {},{},{}".format(kp, ki, kd))

    def set_ff_gain(self, gain):
        """Update feed-forward gain on the Pico."""
        self._send("F {}".format(gain))

    def get_status(self):
        """Get the latest wheel motor status."""
        with self._lock:
            return (
                list(self.targets),
                list(self.rpms),
                list(self.pwms),
                list(self.positions),
            )

    def cleanup(self):
        """Stop motors and close serial connection."""
        try:
            self._send("S")
        except Exception:
            pass
        self._stop_event.set()
        self._read_thread.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:
            pass
        rospy.loginfo("Pico wheel interface closed")
