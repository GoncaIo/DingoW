import rospy
import numpy as np
from dingo_control.State import BehaviorState, State
from dingo_control.Command import Command
from dingo_utilities.Utilities import deadband, clipped_first_order_filter
from sensor_msgs.msg import Joy


class InputInterface:
    def __init__(self, config):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_stagger_toggle = 0
        # Diagnostics for controller button discovery (see input_callback)
        self._reported_button_count = False
        self._seen_unmapped = set()
        self.previous_wheeled_toggle = 0
        self.previous_imu_toggle = 0
        self.previous_crawl_toggle = 0
        self.previous_speed_up_toggle = 0
        self.previous_speed_down_toggle = 0
        self.previous_paw_toggle = 0
        self.previous_low_power_toggle = 0
        self.previous_z_clear_up_toggle = 0
        self.previous_z_clear_down_toggle = 0

        self.rounding_dp = 2

        self.trot_event = 0
        self.wheeled_event = 0
        self.imu_toggle_event = 0
        self.crawl_event = 0
        self.joystick_control_event = 0   # permanently 0: no button bound to it
        self.stagger_toggle_event = 0
        self.speed_up_event = 0
        self.speed_down_event = 0
        self.paw_event = 0
        self.low_power_event = 0
        self.z_clear_up_event = 0
        self.z_clear_down_event = 0

        # Trigger axes can report 0.0 (= half pull) before first touched;
        # only trust each trigger after it has reported its released position.
        self.r2_ready = False
        self.l2_ready = False

        # Raw stick values, updated by input_callback
        self.raw_lx = 0.0
        self.raw_ly = 0.0
        self.raw_rx = 0.0
        self.raw_ry = 0.0

        # Tracks which continuous inputs are currently active, so debug
        # logging fires once on press and once on release instead of at 30 Hz.
        self.active_inputs = {}

        self.input_messages = rospy.Subscriber("joy", Joy, self.input_callback)
        self.current_command = Command()
        self.new_command = Command()
        self.developing_command = Command()

    def input_callback(self, msg):
        self.developing_command = Command()
        ####### Handle discrete commands ########
        # Check if requesting a state transition to trotting, or from trotting to resting
        gait_toggle = msg.buttons[0] #X (cross)
        if self.trot_event != 1:
            self.trot_event = (gait_toggle == 1 and self.previous_gait_toggle == 0)

        # HOP is disabled: the state exists but has no motion code, so entering
        # it would just freeze the robot.

        # Check if requesting a transition to/from pure wheeled driving
        wheeled_toggle = msg.buttons[1] #circle
        if self.wheeled_event != 1:
            self.wheeled_event = (wheeled_toggle == 1 and self.previous_wheeled_toggle == 0)

        # Check if toggling IMU stabilization on/off
        imu_toggle = msg.buttons[2] #triangle
        if self.imu_toggle_event != 1:
            self.imu_toggle_event = (imu_toggle == 1 and self.previous_imu_toggle == 0)

        # Check if requesting a transition to/from the crawl (static walk) gait
        crawl_toggle = msg.buttons[3] #square
        if self.crawl_event != 1:
            self.crawl_event = (crawl_toggle == 1 and self.previous_crawl_toggle == 0)

        # R1/L1 step the active gait's cadence up/down (see Controller.run())
        speed_up_toggle = msg.buttons[5] #R1
        if self.speed_up_event != 1:
            self.speed_up_event = (speed_up_toggle == 1 and self.previous_speed_up_toggle == 0)

        speed_down_toggle = msg.buttons[4] #L1
        if self.speed_down_event != 1:
            self.speed_down_event = (speed_down_toggle == 1 and self.previous_speed_down_toggle == 0)

        paw_toggle = 0
        stagger_toggle = msg.buttons[12] #R3 (right stick click)
        if self.stagger_toggle_event != 1:
            self.stagger_toggle_event = (stagger_toggle == 1 and self.previous_stagger_toggle == 0)

        low_power_toggle = msg.buttons[11] #L3 (left stick click)
        if self.low_power_event != 1:
            self.low_power_event = (low_power_toggle == 1 and self.previous_low_power_toggle == 0)

        z_clear_up_toggle = msg.buttons[9] #options
        if self.z_clear_up_event != 1:
            self.z_clear_up_event = (z_clear_up_toggle == 1 and self.previous_z_clear_up_toggle == 0)

        z_clear_down_toggle = msg.buttons[8] #share
        if self.z_clear_down_event != 1:
            self.z_clear_down_event = (z_clear_down_toggle == 1 and self.previous_z_clear_down_toggle == 0)

        if not self._reported_button_count:
            self._reported_button_count = True
            rospy.loginfo("[INPUT] controller exposes %d buttons (valid indices "
                          "0-%d) and %d axes",
                          len(msg.buttons), len(msg.buttons) - 1, len(msg.axes))
        _bound = {0, 1, 2, 3, 4, 5, 8, 9, 11, 12}
        _reserved = {6: "L2 (also throttle axis)", 7: "R2 (also throttle axis)",
                     10: "PS (launcher start/stop)"}
        for _i, _v in enumerate(msg.buttons):
            if _v == 1 and _i not in _bound and _i not in self._seen_unmapped:
                self._seen_unmapped.add(_i)
                if _i in _reserved:
                    rospy.loginfo("[INPUT] button %d pressed - reserved: %s",
                                  _i, _reserved[_i])
                else:
                    rospy.logwarn("[INPUT] button index %d pressed and bound to "
                                  "NOTHING -- this index is free to use", _i)

        # Debug: announce button presses on their rising edge
        if gait_toggle == 1 and self.previous_gait_toggle == 0:
            rospy.loginfo("[INPUT] X pressed -> toggle trot")
        if wheeled_toggle == 1 and self.previous_wheeled_toggle == 0:
            rospy.loginfo("[INPUT] Circle pressed -> toggle wheeled mode")
        if imu_toggle == 1 and self.previous_imu_toggle == 0:
            rospy.loginfo("[INPUT] Triangle pressed -> toggle IMU stabilization")
        if crawl_toggle == 1 and self.previous_crawl_toggle == 0:
            rospy.loginfo("[INPUT] Square pressed -> toggle crawl gait")
        if speed_up_toggle == 1 and self.previous_speed_up_toggle == 0:
            rospy.loginfo("[INPUT] R1 pressed -> gait speed up")
        if speed_down_toggle == 1 and self.previous_speed_down_toggle == 0:
            rospy.loginfo("[INPUT] L1 pressed -> gait speed down")
        if low_power_toggle == 1 and self.previous_low_power_toggle == 0:
            rospy.loginfo("[INPUT] L3 pressed -> toggle low-power wheeled (servo relax)")
        if stagger_toggle == 1 and self.previous_stagger_toggle == 0:
            rospy.loginfo("[INPUT] R3 pressed -> toggle low-power stagger")
        if z_clear_up_toggle == 1 and self.previous_z_clear_up_toggle == 0:
            rospy.loginfo("[INPUT] Options pressed -> more swing lift")
        if z_clear_down_toggle == 1 and self.previous_z_clear_down_toggle == 0:
            rospy.loginfo("[INPUT] Share pressed -> less swing lift")

        # Update previous values for toggles and state
        self.previous_gait_toggle = gait_toggle
        self.previous_wheeled_toggle = wheeled_toggle
        self.previous_imu_toggle = imu_toggle
        self.previous_crawl_toggle = crawl_toggle
        self.previous_speed_up_toggle = speed_up_toggle
        self.previous_speed_down_toggle = speed_down_toggle
        self.previous_paw_toggle = paw_toggle
        self.previous_low_power_toggle = low_power_toggle
        self.previous_stagger_toggle = stagger_toggle
        self.previous_z_clear_up_toggle = z_clear_up_toggle
        self.previous_z_clear_down_toggle = z_clear_down_toggle

        ####### Handle continuous commands ########
        x_vel = (msg.axes[1] ) * self.config.max_x_velocity #ly
        y_vel = msg.axes[0] * self.config.max_y_velocity #lx
        self.developing_command.horizontal_velocity =  np.round(np.array([x_vel, y_vel]),self.rounding_dp)
        self.developing_command.yaw_rate = np.round(msg.axes[3],self.rounding_dp) * self.config.max_yaw_rate #rx

        self.developing_command.pitch = np.round(msg.axes[4],self.rounding_dp) * self.config.max_pitch #ry
        self.developing_command.height_movement = np.round(msg.axes[7],self.rounding_dp) #dpady
        self.developing_command.roll_movement = -np.round(msg.axes[6],self.rounding_dp) #dpadx

        self.raw_lx = np.round(msg.axes[0], self.rounding_dp)
        self.raw_ly = np.round(msg.axes[1], self.rounding_dp)
        self.raw_rx = np.round(msg.axes[3], self.rounding_dp)
        self.raw_ry = np.round(msg.axes[4], self.rounding_dp)

        if msg.axes[5] > 0.9:
            self.r2_ready = True
        if msg.axes[2] > 0.9:
            self.l2_ready = True
        r2_force = (1.0 - msg.axes[5]) / 2.0 if self.r2_ready else 0.0
        l2_force = (1.0 - msg.axes[2]) / 2.0 if self.l2_ready else 0.0
        # Trigger springs don't always return to exactly +1.0; ignore pulls
        # under 5% so a slightly sticky trigger can't leak throttle.
        if r2_force < 0.05:
            r2_force = 0.0
        if l2_force < 0.05:
            l2_force = 0.0
        self.developing_command.wheel_throttle = np.round(r2_force - l2_force, self.rounding_dp)

        # Debug: announce continuous inputs when they engage/disengage.
        # Thresholds sit above typical DS4 stick drift so idle sticks stay quiet.
        cmd = self.developing_command
        self._log_input("Left stick (walk velocity)",
                        abs(x_vel) > 0.1 or abs(y_vel) > 0.1,
                        "vx=%.2f vy=%.2f m/s" % (x_vel, y_vel))
        self._log_input("Right stick X (yaw rate)",
                        abs(cmd.yaw_rate) > 0.2,
                        "%.2f rad/s" % cmd.yaw_rate)
        self._log_input("Right stick Y (pitch)",
                        abs(cmd.pitch) > 0.05,
                        "%.2f rad" % cmd.pitch)
        self._log_input("D-pad up/down (height)",
                        cmd.height_movement != 0,
                        "%+.0f" % cmd.height_movement)
        self._log_input("D-pad left/right (roll)",
                        cmd.roll_movement != 0,
                        "%+.0f" % cmd.roll_movement)
        self._log_input("R2/L2 (wheel throttle)",
                        abs(cmd.wheel_throttle) > 0.01,
                        "%.2f" % cmd.wheel_throttle)

        self.new_command = self.developing_command

    def _log_input(self, name, active, detail):
        """Log a continuous input once when it engages and once when it releases."""
        was_active = self.active_inputs.get(name, False)
        if active and not was_active:
            rospy.loginfo("[INPUT] %s: %s", name, detail)
        elif was_active and not active:
            rospy.loginfo("[INPUT] %s released", name)
        self.active_inputs[name] = active
        
    def get_command(self, state, message_rate):

        self.current_command = self.new_command

        self.current_command.trot_event = self.trot_event
        self.current_command.wheeled_event = self.wheeled_event
        self.current_command.imu_toggle_event = self.imu_toggle_event
        self.current_command.crawl_event = self.crawl_event
        self.current_command.speed_up_event = self.speed_up_event
        self.current_command.speed_down_event = self.speed_down_event
        self.current_command.paw_event = self.paw_event
        self.current_command.low_power_event = self.low_power_event
        self.current_command.joystick_control_event = self.joystick_control_event
        self.current_command.stagger_toggle_event = self.stagger_toggle_event
        self.current_command.z_clear_up_event = self.z_clear_up_event
        self.current_command.z_clear_down_event = self.z_clear_down_event
        self.trot_event = 0
        self.wheeled_event = 0
        self.imu_toggle_event = 0
        self.crawl_event = 0
        self.speed_up_event = 0
        self.speed_down_event = 0
        self.paw_event = 0
        self.low_power_event = 0
        self.stagger_toggle_event = 0
        self.z_clear_up_event = 0
        self.z_clear_down_event = 0

        message_dt = 1.0 / message_rate

        self.current_command.wheel_yaw_rate = self.raw_rx * self.config.max_yaw_rate

        if state.behavior_state == BehaviorState.WHEELED:
            self.current_command.wheel_throttle = self.raw_ry if abs(self.raw_ry) > 0.1 else 0.0
            if abs(self.raw_rx) < 0.15:
                self.current_command.wheel_yaw_rate = 0.0

            self.current_command.pitch = self.raw_ly * self.config.max_pitch
            self.current_command.yaw_rate = self.raw_lx * self.config.max_yaw_rate
            self.current_command.horizontal_velocity = np.array([0.0, 0.0])

        deadbanded_pitch = deadband(
            self.current_command.pitch, self.config.pitch_deadband
        )
        pitch_rate = clipped_first_order_filter(
            state.pitch,
            deadbanded_pitch,
            self.config.max_pitch_rate,
            self.config.pitch_time_constant,
        )
        self.current_command.pitch  = np.clip(state.pitch + message_dt * pitch_rate, -0.35,0.35)
        self.current_command.height = np.clip(state.height - message_dt * self.config.z_speed * self.current_command.height_movement,-0.24,-0.08)
        self.current_command.roll   = np.clip(state.roll + message_dt * self.config.roll_speed * self.current_command.roll_movement, -0.3,0.3)

        return self.current_command
    