#!/usr/bin/env python3
from dingo_servo_interfacing.lx16a import LX16A, ServoTimeoutError, ServoChecksumError
import numpy as np
import math as m
import rospy
import glob
import os
import time

LOW_POWER_MODE = True
LOW_POWER_STAGGER_GROUPS = [[0, 3], [1, 2]]
LOW_POWER_MOVE_TIME_MS = 80  # glide time so legs keep moving between updates

LOW_POWER_HOLD_CYCLES = 2


USE_PARALLELOGRAM_LOWER = True
PARA_GAIN = 1.8827                       # deg of servo per deg of link angle
PARA_OFFSETS = np.array([52.0,           # FR
                         223.3,          # FL
                         45.2,           # BR
                         207.9])         # BL


class HardwareInterface():
    def __init__(self, link, port="/dev/ttyUSB0"):
        self.link = link
        self.servo_angles = np.zeros((3, 4))
        self.port = port
        self._last_reconnect_attempt = 0.0
        self._just_reconnected = False
        self._stagger_index = 0  # TEMPORARY low-power mode group rotation
        self._stagger_hold = 0   # cycles the current group has been held
        self.reconnect_count = 0
        self.last_bus_error = ""
        # Per-instance so it can be toggled at runtime (R3 on the pad, or the
        # WebTuner page). Seeded from the module constants above.
        self.low_power_mode = LOW_POWER_MODE
        self.stagger_hold_cycles = LOW_POWER_HOLD_CYCLES
        self.low_power_move_time_ms = LOW_POWER_MOVE_TIME_MS

        try:
            LX16A.initialize(port, timeout=0.05)
        except Exception as e:
            rospy.logerr(f"Failed to initialize LX16A on port {port}: {e}")
            raise

        self.servo_ids = np.array([[0, 3, 6, 9],
                                    [1, 4, 7, 10],
                                    [2, 5, 8, 11]])

        self.directions = np.array(
                            [[1, -1, -1, 1],
                            [1, -1, 1, -1],
                            [1, -1, 1, -1]])

        self.offsets = np.array(
                    [[118.6, 111.8, 129.4, 121.0],
                    [32.6, 93.8, 35.5, 187.4],
                    [51.8, 217.2, 44.9, 294.2]])

        self.min_limits = np.array(
                    [[93, 67, 97, 91],
                    [15, 4, 15, 76],
                    [56, 78, 30, 73]])
        self.max_limits = np.array(
                    [[159, 140, 161, 158],
                    [138, 116, 146, 224],
                    [185, 215, 177, 235]])

        if USE_PARALLELOGRAM_LOWER:
            missing = [n for n, v in zip(("FR", "FL", "BR", "BL"), PARA_OFFSETS)
                       if not np.isfinite(v)]
            if missing:
                raise RuntimeError(
                    "PARA_OFFSETS not measured for: %s. The parallelogram "
                    "lower-joint model needs a per-leg zero and it CANNOT be "
                    "derived from another leg (doing so puts BL 45 deg past "
                    "its stop). Measure each with 'caloffset <leg>' in "
                    "w_dingo/AltIK.py, paste the values into PARA_OFFSETS, or "
                    "set USE_PARALLELOGRAM_LOWER = False to fall back to the "
                    "old four-bar." % ", ".join(missing))

        # Initialize servos
        self.create()

    def create(self):
        """Initialize LX16A servo objects with proper configuration."""
        self.servo_objects = {}
        # Force a fresh send to every servo after (re)initialization
        self._last_sent_angles = {}
        for leg_index in range(4):
            for axis_index in range(3):
                servo_id = self.servo_ids[axis_index, leg_index]
                try:
                    servo = LX16A(servo_id, disable_torque=False)
                    servo.enable_torque()
                    servo.servo_mode()
                    self.servo_objects[servo_id] = servo
                    rospy.loginfo(f"Initialized servo {servo_id} (axis {axis_index}, leg {leg_index})")
                except (ServoTimeoutError, ServoChecksumError) as e:
                    rospy.logwarn(f"Warning initializing servo {servo_id}: {e}")
                except Exception as e:
                    rospy.logwarn(f"Warning initializing servo {servo_id}: {e}")

    def set_actuator_postions(self, joint_angles, move_time_ms=50):
        """Converts all angles found via inverse kinematics to the angles needed at the servo by applying multipliers"""
        if self._just_reconnected:
            move_time_ms = max(move_time_ms, 500)
            self._just_reconnected = False

        # Limit angles to physical possibility
        possible_joint_angles = impose_physical_limits(joint_angles)

        # Convert to servo angles
        self.joint_angles_to_servo_angles(possible_joint_angles)

        if self.low_power_mode and move_time_ms < self.low_power_move_time_ms:
            legs_to_send = LOW_POWER_STAGGER_GROUPS[self._stagger_index]
            # Hold each group for stagger_hold_cycles control cycles before
            # moving on, rather than switching every cycle.
            self._stagger_hold += 1
            if self._stagger_hold >= max(1, int(self.stagger_hold_cycles)):
                self._stagger_hold = 0
                self._stagger_index = (self._stagger_index + 1) % len(LOW_POWER_STAGGER_GROUPS)
            move_time_ms = self.low_power_move_time_ms
        else:
            legs_to_send = range(4)

        # Send servo commands
        attempted = 0
        failed = 0
        for leg_index in legs_to_send:
            for axis_index in range(3):
                servo_id = self.servo_ids[axis_index, leg_index]
                angle = self.servo_angles[axis_index, leg_index]

                if servo_id not in self.servo_objects:
                    rospy.logwarn(f"Servo {servo_id} not initialized, skipping")
                    continue

                if self._last_sent_angles.get(servo_id) == angle:
                    continue

                attempted += 1
                try:
                    self.servo_objects[servo_id].move(angle, time=move_time_ms)
                    self._last_sent_angles[servo_id] = angle
                except (ServoTimeoutError, ServoChecksumError) as e:
                    self._last_sent_angles.pop(servo_id, None)
                    rospy.logwarn(f"Servo {servo_id} error: {e}")
                except Exception as e:
                    failed += 1
                    self._last_sent_angles.pop(servo_id, None)
                    self.last_bus_error = "servo %d: %s" % (servo_id, e)
                    rospy.logwarn_throttle(1, f"Servo {servo_id} error: {e}")

        # If every write failed the serial device itself is gone (e.g. BusLinker
        # brownout during a current spike) — try to reopen the bus.
        if attempted > 0 and failed == attempted:
            self._attempt_reconnect()

    def _attempt_reconnect(self):
        """Reopen the servo bus after the USB device dropped and re-enumerated."""
        now = time.monotonic()
        if now - self._last_reconnect_attempt < 1.0:
            return
        self._last_reconnect_attempt = now

        # Release the stale handle first so the kernel can free the old name
        if LX16A._controller is not None:
            try:
                LX16A._controller.close()
            except Exception:
                pass
            LX16A._controller = None

        # Try every ttyUSB node: after a brownout the board may re-enumerate
        # under a new name while a stale node with the old name lingers.
        candidates = sorted(glob.glob("/dev/ttyUSB*"))
        if self.port in candidates:
            candidates.remove(self.port)
            candidates.insert(0, self.port)
        if not candidates:
            rospy.logwarn_throttle(5, "Servo bus disconnected, waiting for it to reappear...")
            return

        for candidate in candidates:
            try:
                LX16A.initialize(candidate, timeout=0.05)
                self.port = candidate
                self.reconnect_count += 1
                rospy.logwarn(f"Servo bus reconnected on {candidate}, re-initializing servos")
                self.create()
                self._just_reconnected = True
                return
            except Exception as e:
                rospy.logwarn_throttle(5, f"Servo bus reconnect to {candidate} failed: {e}")

    def relax_all_motors(self, servo_list=np.ones((3, 4))):
        """Relaxes desired servos so that they appear to be turned off."""
        for leg_index in range(4):
            for axis_index in range(3):
                if servo_list[axis_index, leg_index] == 1:
                    servo_id = self.servo_ids[axis_index, leg_index]
                    if servo_id in self.servo_objects:
                        try:
                            self.servo_objects[servo_id].disable_torque()
                        except Exception as e:
                            rospy.logwarn(f"Error relaxing servo {servo_id}: {e}")

    def engage_all_motors(self):
        """Re-enable torque on all servos after relax_all_motors()."""
        self._last_sent_angles = {}
        for servo_id, servo in self.servo_objects.items():
            try:
                servo.enable_torque()
            except Exception as e:
                rospy.logwarn(f"Error engaging servo {servo_id}: {e}")

    def joint_angles_to_servo_angles(self, joint_angles):
        """Converts joint found via inverse kinematics to the angles needed at the servo using linkage analysis."""

        for leg in range(4):
            THETA2, THETA3 = joint_angles[1:, leg]

            # Adding offset from IK angle definition to servo angle definition, and conversion to degrees
            self.servo_angles[0, leg] = m.degrees(joint_angles[0, leg])
            self.servo_angles[1, leg] = m.degrees(THETA2)
            if USE_PARALLELOGRAM_LOWER:
                # filled in below; the parallelogram model does not use the
                # offset/direction pair the same way the other two joints do
                self.servo_angles[2, leg] = 0.0
            else:
                THETA0 = lower_leg_angle_to_servo_angle(self.link, m.pi/2-THETA2, THETA3 + np.pi/2)
                self.servo_angles[2, leg] = m.degrees(m.pi/2 + m.pi-THETA0)

        # Apply calibration: servo_angle = offset + direction * angle
        self.servo_angles = np.round(
            self.offsets + np.multiply(self.servo_angles, self.directions), 1
        )

        if USE_PARALLELOGRAM_LOWER:
            lam = np.degrees(joint_angles[1, :]) + np.degrees(joint_angles[2, :])
            self.servo_angles[2, :] = np.round(
                PARA_OFFSETS + self.directions[2, :] * PARA_GAIN * lam, 1
            )

        self.servo_angles = np.clip(self.servo_angles, self.min_limits, self.max_limits)


### FUNCTIONS ###

def calculate_4_bar(th2, a, b, c, d):
    """Using 'Freudensteins method', it finds all the angles within a 4 bar linkage with vertices ABCD and known link lengths a,b,c,d"""
    x_b = a*np.cos(th2)
    y_b = a*np.sin(th2)

    # define diagonal f
    f = np.sqrt((d-x_b)**2 + y_b**2)
    beta = np.arccos((f**2+c**2-b**2)/(2*f*c))
    gamma = np.arctan2(y_b, d-x_b)

    th4 = np.pi - gamma - beta

    x_c = c*np.cos(th4)+d
    y_c = c*np.sin(th4)

    th3 = np.arctan2((y_c-y_b), (x_c-x_b))

    ## Calculate remaining internal angles of linkage
    ABC = np.pi-th2 + th3
    BCD = th4-th3
    CDA = np.pi*2 - th2 - ABC - BCD

    return ABC, BCD, CDA


def lower_leg_angle_to_servo_angle(link, THETA2, THETA3):
    """Converts the direct angles of the upper and lower leg joint from the inverse kinematics to the angle"""

    # First 4 bar linkages
    GDE, DEF, EFG = calculate_4_bar(THETA3 + link.lower_leg_bend_angle, link.i, link.h, link.f, link.g)
    # Triangle section
    CDH = 3/2*m.pi - THETA2 - GDE - link.EDC
    CDA = CDH + link.gamma  # input angle
    # Second 4 bar linkage
    DAB, ABC, BCD = calculate_4_bar(CDA, link.d, link.a, link.b, link.c)
    # Calculating Theta
    THETA0 = DAB + link.gamma

    return THETA0


def impose_physical_limits(desired_joint_angles):
    """Takes desired upper and lower leg angles and clips them to be within the range of physical possibility."""
    possible_joint_angles = np.zeros((3, 4))

    for i in range(4):
        hip, upper, lower = np.degrees(desired_joint_angles[:, i])

        hip = np.clip(hip, -20, 20)
        upper = np.clip(upper, 0, 120)

        if      0    <=  upper <     10:
            lower = np.clip(lower, -20, 40)
        elif 10    <=  upper <     20:
            lower = np.clip(lower, -40, 40)
        elif 20    <=  upper <     30:
            lower = np.clip(lower, -50, 40)
        elif 30    <=  upper <     40:
            lower = np.clip(lower, -60, 30)
        elif 40    <=  upper <     50:
            lower = np.clip(lower, -70, 25)
        elif 50    <=  upper <     60:
            lower = np.clip(lower, -70, 20)
        elif 60    <=  upper <     70:
            lower = np.clip(lower, -70, 0)
        elif 70    <=  upper <     80:
            lower = np.clip(lower, -70, -10)
        elif 80    <=  upper <     90:
            lower = np.clip(lower, -70, -20)
        elif 90    <=  upper <     100:
            lower = np.clip(lower, -70, -30)
        elif 100    <=  upper <     110:
            lower = np.clip(lower, -70, -40)
        elif 110    <=  upper <     120:
            lower = np.clip(lower, -70, -60)

        possible_joint_angles[:, i] = hip, upper, lower

    return np.radians(possible_joint_angles)
