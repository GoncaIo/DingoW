#!/usr/bin/env python3
# coding: utf-8
import sys
import numpy as np
import time
import os
import platform

if os.name == "nt":
    import msvcrt
else:
    import tty
    import termios

from dingo_servo_interfacing.lx16a import LX16A

r'''    
HOW TO CALIBRATE LX16A SERVOS FOR DINGO
This is how the robot should look at the calibration position of [0,0,90]
                            LINKAGE
                          /‾‾‾‾‾‾‾\------------------- q
                         /   _______                   |
                        |   |    o__|___UPPER LEG______/   <---- UPPER LEG AT 0° POINTS HORIZONTALLY BACKWARD
   LOWER LEG SERVO -->  |___|__o    |‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾/
      AT 90° POINTS         |       |                /
   HORIZONTALLY FORWARD      ‾‾‾‾‾‾‾                /
                            SERVO HUB              / LOWER LEG
                                                  /
                                                 O

CALIBRATION PROCESS:
1. Mount servo hubs (without legs) on hip servos at approximately 90 degrees
2. Connect LX16A servos to /dev/ttyUSB0 with IDs 0-11
3. Run this script with all servos disabled initially
4. For each servo:
   - Enable torque and move to calibration position
   - Mount legs such that positive angles achieve desired position
   - Adjust offset values until home position is correct
5. Copy final offset values to HardwareInterface_LX16A.py

SERVO ID MAPPING:
  0 = Front-Right Hip       6 = Back-Right Hip
  1 = Front-Right Upper     7 = Back-Right Upper
  2 = Front-Right Lower     8 = Back-Right Lower
  3 = Front-Left Hip        9 = Back-Left Hip
  4 = Front-Left Upper     10 = Back-Left Upper
  5 = Front-Left Lower     11 = Back-Left Lower
'''

# Calibration position: [hip, upper, lower]
calibration_pos = [0, 0, 90]
low_pos = [0, 25, 140] 
mid_pos = [0, 42, 120]
high_pos = [0, 50, 110]

car_position = [0, -10, 140]

hip_open_pos = [20, 0, 90]
hip_closed_pos = [-20, 0, 90]

offsets = np.array([
    [118.6,     111.8,   129.4,      121.0],     # Hip offsets:   FR, FL, BR, BL
    [32.6,       93.8,    35.5,      187.4],     # Upper offsets: FR, FL, BR, BL
    [51.8,      217.2,    44.9,      294.2]      # Lower offsets: FR, FL, BR, BL
])

directions = np.array([
    [1, -1, -1, 1],       # Hip: front (FR,FL) turn CCW, back (BR,BL) turn CW - opposite sense
    [1, -1, 1, -1],      # Upper: FR, BR = normal; FL, BL = reversed
    [1, -1, 1, -1]       # Lower: FR, BR = normal; FL, BL = reversed
])

servo_ids = np.array([
    [0, 3, 6, 9],           # Hip servos
    [1, 4, 7, 10],          # Upper servos
    [2, 5, 8, 11]           # Lower servos
])

servo_limits = np.array([
    [93, 159],    # 0  FR hip
    [15, 138],    # 1  FR upper
    [56, 185],    # 2  FR lower
    [67, 140],    # 3  FL hip
    [4, 116],     # 4  FL upper
    [78, 215],    # 5  FL lower
    [97, 161],    # 6  BR hip
    [15, 146],    # 7  BR upper
    [30, 177],    # 8  BR lower
    [91, 158],    # 9  BL hip
    [76, 224],    # 10 BL upper
    [73, 235],    # 11 BL lower
])

servo_objects = {}


def read_key():
    """Cross-platform key input"""
    if os.name == "nt":
        key = msvcrt.getch()
        if key in (b"\x00", b"\xe0"):
            msvcrt.getch()
            return None
        return key.decode(errors="ignore").lower()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1).lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def initialize_servos():
    """Initialize all 12 LX16A servos"""
    global servo_objects
    
    # Detect OS and choose appropriate port
    if platform.system() == "Windows":
        port = "COM6"  # Change to COM4, COM5, etc. if needed
    else:
        port = "/dev/ttyUSB0"
    
    try:
        # Use shorter timeout for initialization
        LX16A.initialize(port, 115200)
        LX16A.set_timeout(0.05)  # 50ms timeout
        print(f"✓ LX16A bus initialized at {port}")

        for servo_id in range(12):
            try:
                servo = LX16A(servo_id)
                servo_objects[servo_id] = servo
                servo.disable_torque()
                print(f"✓ Servo {servo_id} initialized (torque disabled)")
            except Exception as e:
                print(f"✗ Servo {servo_id} NOT responding")
                continue

        return True
    except Exception as e:
        print(f"✗ Failed to initialize LX16A bus at {port}: {e}")
        if platform.system() == "Windows":
            print("\nTroubleshooting:")
            print("1. Check COM port: run PowerShell command:")
            print("   [System.IO.Ports.SerialPort]::GetPortNames()")
            print("2. Update 'port = \"COM6\"' in this script to the correct port")
        return False


def move_servo(servo_id, angle):
    """Move a single servo to angle, clamped to its safe calibrated range"""
    min_angle, max_angle = servo_limits[servo_id]
    if angle < min_angle or angle > max_angle:
        print(f"⚠ Servo {servo_id} angle {angle:.1f}° out of range [{min_angle}, {max_angle}], clamping")
    angle = np.clip(angle, min_angle, max_angle)

    try:
        servo_objects[servo_id].move(angle, time=200)
    except Exception as e:
        print(f"✗ Error moving servo {servo_id}: {e}")


def move_leg(leg_name, pos):
    """Move a complete leg to [hip, upper, lower] angles"""
    leg_map = {
        "fr": 0,  # Front-Right
        "fl": 1,  # Front-Left
        "br": 2,  # Back-Right
        "bl": 3   # Back-Left
    }

    if leg_name not in leg_map:
        print(f"Invalid leg: {leg_name}. Use: fr, fl, br, bl")
        return

    leg_idx = leg_map[leg_name]

    # Move all three joints
    for joint in range(3):  # 0=hip, 1=upper, 2=lower
        servo_id = servo_ids[joint, leg_idx]
        # Apply direction multiplier to flip servo if needed
        direction = directions[joint, leg_idx]
        angle = offsets[joint, leg_idx] + direction * pos[joint]
        move_servo(servo_id, angle)
        print(f"  {['Hip', 'Upper', 'Lower'][joint]:6} -> servo {servo_id:2} at {angle:.1f}°")


def move_all_legs(pos):
    """Move all four legs to same position"""
    legs = ["fr", "fl", "br", "bl"]
    print(f"Moving all legs to {pos}:")
    for leg in legs:
        move_leg(leg, pos)


def relax_all_servos():
    """Disable all servos (release torque)"""
    for servo_id in range(12):
        try:
            servo_objects[servo_id].disable_torque()
        except:
            pass
    print("✓ All servos relaxed (torque disabled)")


def enable_servo_torque(servo_id):
    """Enable torque on a single servo"""
    try:
        servo_objects[servo_id].enable_torque()
        print(f"✓ Servo {servo_id} torque enabled")
    except Exception as e:
        print(f"✗ Error enabling servo {servo_id}: {e}")


def enable_all_torque():
    """Enable torque on all servos"""
    for servo_id in range(12):
        try:
            servo_objects[servo_id].enable_torque()
        except:
            pass
    print("✓ All servos torque enabled")


def get_servo_angles():
    """Read current angles from all servos"""
    angles = {}
    for servo_id in range(12):
        try:
            angle = servo_objects[servo_id].get_physical_angle()
            angles[servo_id] = angle
        except:
            angles[servo_id] = None
    return angles


def print_servo_status():
    """Read and print live voltage, temperature and error flags for all servos.
    Useful for diagnosing high-current/stall issues that only appear under load
    (no-load calibration can't reveal torque/power problems)."""
    print("\n" + "="*60)
    print("Servo Status (input voltage, temperature, error flags):")
    print("="*60)
    for servo_id in range(12):
        try:
            vin_mv = servo_objects[servo_id].get_vin()
            temp = servo_objects[servo_id].get_temp()
            over_temp, over_volt, rotor_locked = servo_objects[servo_id].get_led_error_triggers(poll_hardware=True)
            flags = []
            if over_temp:
                flags.append("OVER-TEMP")
            if over_volt:
                flags.append("OVER-VOLTAGE")
            if rotor_locked:
                flags.append("STALLED/ROTOR-LOCKED")
            flag_str = ", ".join(flags) if flags else "ok"
            print(f"  Servo {servo_id:2}: {vin_mv/1000:5.2f} V   {temp:3}°C   [{flag_str}]")
        except Exception as e:
            print(f"  Servo {servo_id:2}: ERROR reading status ({e})")
    print("="*60 + "\n")


def adjust_offset(joint, leg, value):
    """Adjust an offset value"""
    joint_names = {0: "Hip", 1: "Upper", 2: "Lower"}
    leg_names = {0: "FR", 1: "FL", 2: "BR", 3: "BL"}

    if 0 <= joint < 3 and 0 <= leg < 4:
        offsets[joint, leg] += value
        print(f"Adjusted {joint_names[joint]} offset for {leg_names[leg]} to {offsets[joint, leg]}")
        return True
    return False


def print_offsets():
    """Print current offset matrix in NumPy format"""
    print("\n" + "="*60)
    print("Current Calibration Offsets:")
    print("="*60)
    print("offsets = np.array([")
    for row in offsets:
        print(f"    [{', '.join(f'{x:3.0f}' for x in row)}],")
    print("])")
    print("="*60 + "\n")


def print_directions():
    """Print current direction matrix"""
    print("\n" + "="*60)
    print("Current Servo Directions (1=normal, -1=reversed):")
    print("="*60)
    print("directions = np.array([")
    for row in directions:
        print(f"    [{', '.join(f'{int(x):2}' for x in row)}],")
    print("])")
    print("="*60 + "\n")


def toggle_direction(joint, leg):
    """Toggle servo direction (1 to -1 or vice versa)"""
    joint_names = {0: "Hip", 1: "Upper", 2: "Lower"}
    leg_names = {0: "FR", 1: "FL", 2: "BR", 3: "BL"}

    if 0 <= joint < 3 and 0 <= leg < 4:
        directions[joint, leg] *= -1
        direction_str = "normal" if directions[joint, leg] == 1 else "reversed"
        print(f"Toggled {joint_names[joint]} direction for {leg_names[leg]} to {direction_str}")
        return True
    return False


def print_help():
    """Print available commands"""
    print("\n" + "="*60)
    print("CALIBRATION COMMANDS:")
    print("="*60)
    print("Torque Control:")
    print("  enable all          - Enable torque on all servos")
    print("  enable <id>         - Enable torque on servo (0-11)")
    print("  relax               - Disable torque on all servos")
    print("")
    print("Position Control:")
    print("  all cal     - Move all legs to calibration [0, 0, 90]")
    print("  all low     - Move all legs to low stance [0, 25, 140]")
    print("  all mid     - Move all legs to mid stance [0, 42, 120]")
    print("  all high    - Move all legs to high stance [0, 50, 110]")
    print("  all car     - Move all legs to car position [0, -10, 140]")
    print("  all hipopen   - Swing all hips to +20 (test hip open limit)")
    print("  all hipclosed - Swing all hips to -20 (test hip closed limit)")
    print("  <leg> cal   - Move single leg (fr/fl/br/bl)")
    print("  <leg> low/mid/high/car/hipopen/hipclosed")
    print("")
    print("Individual Servo Control:")
    print("  servo <id> <angle>  - Move servo by ID (0-11) to angle (0-240)")
    print("  read                - Read all servo angles")
    print("  status              - Read voltage/temperature/error flags (diagnose stalls under load)")
    print("")
    print("Offset Adjustment (when leg position is wrong):")
    print("  offset <joint> <leg> <+/-value>")
    print("    joint: 0=hip, 1=upper, 2=lower")
    print("    leg: 0=FR, 1=FL, 2=BR, 3=BL")
    print("    value: positive or negative adjustment")
    print("    Example: offset 0 0 +5  (increase FR hip offset by 5)")
    print("")
    print("Direction Reversal (when servo rotates wrong way):")
    print("  reverse <joint> <leg>  - Toggle servo direction (1 to -1 or vice versa)")
    print("    Example: reverse 0 1  (flip Front-Left Hip direction)")
    print("")
    print("Utilities:")
    print("  offsets     - Print current offset matrix")
    print("  directions  - Print current direction matrix")
    print("  help        - Show this menu")
    print("  quit        - Exit program")
    print("="*60 + "\n")


def main():
    print(__doc__)

    # Initialize
    if not initialize_servos():
        print("Failed to initialize servos. Exiting.")
        return

    print_help()

    try:
        while True:
            try:
                cmd = input("\nDingo_Calibrate> ").strip().split()

                if not cmd:
                    continue

                if cmd[0] == "quit":
                    break

                elif cmd[0] == "help":
                    print_help()

                elif cmd[0] == "offsets":
                    print_offsets()

                elif cmd[0] == "directions":
                    print_directions()

                elif cmd[0] == "relax":
                    relax_all_servos()

                elif cmd[0] == "enable":
                    if len(cmd) >= 2:
                        if cmd[1] == "all":
                            enable_all_torque()
                        else:
                            try:
                                servo_id = int(cmd[1])
                                enable_servo_torque(servo_id)
                            except ValueError:
                                print("Usage: enable <id> or enable all")
                    else:
                        print("Usage: enable <id> or enable all")

                elif cmd[0] == "read":
                    angles = get_servo_angles()
                    print("Current servo angles:")
                    for sid in range(12):
                        if angles[sid] is not None:
                            print(f"  Servo {sid:2}: {angles[sid]:6.1f}°")
                        else:
                            print(f"  Servo {sid:2}: ERROR")

                elif cmd[0] == "status":
                    print_servo_status()

                elif cmd[0] == "servo" and len(cmd) >= 3:
                    servo_id = int(cmd[1])
                    angle = float(cmd[2])
                    move_servo(servo_id, angle)
                    print(f"Moved servo {servo_id} to {angle:.1f}°")

                elif cmd[0] == "offset" and len(cmd) >= 4:
                    joint = int(cmd[1])
                    leg = int(cmd[2])
                    value = float(cmd[3])
                    if adjust_offset(joint, leg, value):
                        move_all_legs(calibration_pos)

                elif cmd[0] == "reverse" and len(cmd) >= 3:
                    joint = int(cmd[1])
                    leg = int(cmd[2])
                    if toggle_direction(joint, leg):
                        move_all_legs(calibration_pos)

                elif cmd[0] == "all" and len(cmd) >= 2:
                    pos_map = {
                        "cal": calibration_pos,
                        "low": low_pos,
                        "mid": mid_pos,
                        "high": high_pos,
                        "car": car_position,
                        "hipopen": hip_open_pos,
                        "hipclosed": hip_closed_pos
                    }
                    if cmd[1] in pos_map:
                        move_all_legs(pos_map[cmd[1]])
                    else:
                        print(f"Unknown position: {cmd[1]}. Use: cal, low, mid, high, car, hipopen, hipclosed")

                elif cmd[0] in ["fr", "fl", "br", "bl"] and len(cmd) >= 2:
                    pos_map = {
                        "cal": calibration_pos,
                        "low": low_pos,
                        "mid": mid_pos,
                        "high": high_pos,
                        "car": car_position,
                        "hipopen": hip_open_pos,
                        "hipclosed": hip_closed_pos
                    }
                    if cmd[1] in pos_map:
                        move_leg(cmd[0], pos_map[cmd[1]])
                    else:
                        print(f"Unknown position: {cmd[1]}. Use: cal, low, mid, high, car, hipopen, hipclosed")

                else:
                    print("Unknown command. Type 'help' for options.")

            except ValueError as e:
                print(f"Invalid input: {e}")
            except Exception as e:
                print(f"Error: {e}")

    except KeyboardInterrupt:
        print("\nCalibration interrupted.")

    finally:
        print("Relaxing all servos...")
        relax_all_servos()
        print("Calibration tool closed.")


if __name__ == "__main__":
    main()
