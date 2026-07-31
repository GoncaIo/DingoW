#!/usr/bin/env python3
# coding: utf-8
"""Record the REAL mechanical travel limits of each leg servo, by hand."""
import sys
import time

import numpy as np

try:  # running inside the ROS package
    from dingo_servo_interfacing.lx16a import LX16A
except ImportError:  # running standalone (e.g. from w_dingo/)
    from lx16a import LX16A


# Degrees to pull each recorded extreme inward, so the software never commands
# the joint hard against its physical stop.
SAFETY_MARGIN_DEG = 3.0

# Readings outside this are treated as bus noise / bad reads and discarded.
VALID_ANGLE_RANGE = (0.0, 240.0)

# A joint whose recorded span is narrower than this probably was not moved.
SUSPICIOUS_SPAN_DEG = 10.0

REFRESH_HZ = 5.0

SERVO_NAMES = {
    0: "FR hip",   1: "FR upper",  2: "FR lower",
    3: "FL hip",   4: "FL upper",  5: "FL lower",
    6: "BR hip",   7: "BR upper",  8: "BR lower",
    9: "BL hip",  10: "BL upper", 11: "BL lower",
}

# [joint, leg] -> servo id, matching HardwareInterface_LX16A.py
SERVO_IDS = np.array([[0, 3, 6, 9],
                      [1, 4, 7, 10],
                      [2, 5, 8, 11]])


def initialise_bus():
    """Open the servo bus and return {servo_id: LX16A}, all with torque OFF."""
    import platform
    port = "COM6" if platform.system() == "Windows" else "/dev/ttyUSB0"
    try:
        LX16A.initialize(port, 115200)
        LX16A.set_timeout(0.05)
    except Exception as exc:
        print("Failed to open the servo bus at %s: %s" % (port, exc))
        return None
    print("Servo bus open at %s" % port)

    servos = {}
    for servo_id in range(12):
        try:
            servo = LX16A(servo_id)
            servo.disable_torque()
            servos[servo_id] = servo
        except Exception:
            print("  servo %2d (%-8s) NOT RESPONDING - it will be skipped"
                  % (servo_id, SERVO_NAMES[servo_id]))
    if not servos:
        print("No servos responded. Check power and the USB connection.")
        return None
    print("%d/12 servos found, torque disabled - the legs should now move "
          "freely by hand." % len(servos))
    return servos


def read_angles(servos):
    """Current angle of every responding servo, skipping bad reads."""
    angles = {}
    for servo_id, servo in servos.items():
        try:
            angle = servo.get_physical_angle()
        except Exception:
            continue
        if angle is None:
            continue
        if VALID_ANGLE_RANGE[0] <= angle <= VALID_ANGLE_RANGE[1]:
            angles[servo_id] = float(angle)
    return angles


def draw(servos, current, lo, hi, samples):
    sys.stdout.write("\033[H\033[J")  # home + clear
    print("RECORDING SERVO LIMITS - move each joint slowly to its stops")
    print("Ctrl-C when finished.    samples: %d" % samples)
    print("=" * 62)
    print("%-4s%-10s%10s%10s%10s%10s"
          % ("id", "joint", "now", "min", "max", "span"))
    print("-" * 62)
    for servo_id in range(12):
        if servo_id not in servos:
            print("%-4d%-10s%40s" % (servo_id, SERVO_NAMES[servo_id], "(no response)"))
            continue
        now = current.get(servo_id)
        now_s = "%.1f" % now if now is not None else "--"
        if lo[servo_id] is None:
            print("%-4d%-10s%10s%10s%10s%10s" % (servo_id, SERVO_NAMES[servo_id],
                                                 now_s, "--", "--", "--"))
            continue
        span = hi[servo_id] - lo[servo_id]
        flag = "  <- move it" if span < SUSPICIOUS_SPAN_DEG else ""
        print("%-4d%-10s%10s%10.1f%10.1f%10.1f%s"
              % (servo_id, SERVO_NAMES[servo_id], now_s,
                 lo[servo_id], hi[servo_id], span, flag))
    print("=" * 62)


def emit_results(lo, hi):
    """Print the recorded limits, margin applied, formatted for both files."""
    missing = [s for s in range(12) if lo[s] is None]
    narrow = [s for s in range(12)
              if lo[s] is not None and (hi[s] - lo[s]) < SUSPICIOUS_SPAN_DEG]

    print("\n" + "=" * 70)
    print("RESULTS  (recorded extremes pulled inward by %.1f deg)" % SAFETY_MARGIN_DEG)
    print("=" * 70)

    if missing:
        print("\n!! NO DATA for: %s" % ", ".join(
            "%d %s" % (s, SERVO_NAMES[s]) for s in missing))
        print("   Those servos did not respond. The arrays below leave them at 0")
        print("   -- do NOT paste this until every joint has been recorded.")
    if narrow:
        print("\n!! SUSPICIOUSLY NARROW (<%.0f deg), probably not moved through"
              " full travel:" % SUSPICIOUS_SPAN_DEG)
        for s in narrow:
            print("     %2d %-9s span %.1f deg" % (s, SERVO_NAMES[s], hi[s] - lo[s]))

    unusable = sorted(set(missing) | set(narrow))
    if unusable:
        print("\n\n*** NOT emitting pasteable arrays: %d joint(s) lack a usable"
              " measurement." % len(unusable))
        for s in unusable:
            span = "no data" if lo[s] is None else "span %.1f deg" % (hi[s] - lo[s])
            print("      %2d %-9s %s" % (s, SERVO_NAMES[s], span))
        print("\n    Re-run and sweep those joints fully. If you only intended to")
        print("    measure some joints this run, that is fine -- take the raw")
        print("    extremes below and merge them into the existing arrays by hand,")
        print("    rather than replacing joints you did not measure.\n")
    else:
        safe_lo, safe_hi = {}, {}
        for s in range(12):
            a = lo[s] + SAFETY_MARGIN_DEG
            b = hi[s] - SAFETY_MARGIN_DEG
            if a > b:  # span smaller than twice the margin
                a = b = (lo[s] + hi[s]) / 2.0
            safe_lo[s] = int(np.ceil(a))
            safe_hi[s] = int(np.floor(b))

        print("\n\n--- paste into CalibrateServos_LX16A.py (indexed by servo id) ---\n")
        print("servo_limits = np.array([")
        for s in range(12):
            cell = "[%d, %d]," % (safe_lo[s], safe_hi[s])
            print("    %-14s# %-2d %s" % (cell, s, SERVO_NAMES[s]))
        print("])")

        print("\n\n--- paste into HardwareInterface_LX16A.py ([joint, leg]) ---\n")
        for name, table in (("min_limits", safe_lo), ("max_limits", safe_hi)):
            rows = ["            [" + ", ".join(
                str(table[SERVO_IDS[j, l]]) for l in range(4)) + "]"
                for j in range(3)]
            # note the doubled brackets: np.array takes ONE nested list
            print("self.%s = np.array(" % name)
            print("            [" + ",\n            ".join(
                r.strip() for r in rows) + "])")
            print()

    print("Raw recorded extremes, before the margin, for reference:")
    for s in range(12):
        if lo[s] is None:
            print("  %2d %-9s  no data" % (s, SERVO_NAMES[s]))
        else:
            print("  %2d %-9s  %6.1f .. %6.1f   (span %.1f)"
                  % (s, SERVO_NAMES[s], lo[s], hi[s], hi[s] - lo[s]))


def main():
    print(__doc__)
    servos = initialise_bus()
    if servos is None:
        return

    print("\nStarting in 2 s - make sure no leg is carrying weight.")
    time.sleep(2.0)

    lo = {s: None for s in range(12)}
    hi = {s: None for s in range(12)}
    samples = 0
    period = 1.0 / REFRESH_HZ

    try:
        while True:
            current = read_angles(servos)
            for servo_id, angle in current.items():
                if lo[servo_id] is None:
                    lo[servo_id] = hi[servo_id] = angle
                else:
                    lo[servo_id] = min(lo[servo_id], angle)
                    hi[servo_id] = max(hi[servo_id], angle)
            samples += 1
            draw(servos, current, lo, hi, samples)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        # Torque was never enabled; leave the servos limp and say so.
        print("\n\nStopped. Servos remain limp (torque was never enabled).")
        emit_results(lo, hi)


if __name__ == "__main__":
    main()
