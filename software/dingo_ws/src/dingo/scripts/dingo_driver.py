import numpy as np
import time
import rospy
import sys
from std_msgs.msg import Float64
import signal
import socket
import platform


#Fetching is_sim and is_physical from arguments
args = rospy.myargv(argv=sys.argv)
if len(args) != 4: #arguments have not been provided, go to defaults (not sim, is physical)
    is_sim = 0
    is_physical = 1
    use_imu = 1
else:
    is_sim = int(args[1])
    is_physical = int(args[2])
    use_imu = int(args[3])

from dingo_control.Controller import Controller
from dingo_input_interfacing.InputInterface import InputInterface
from dingo_input_interfacing.DS4Led import DS4Led
from dingo_control.State import State, BehaviorState
from dingo_control.Kinematics import four_legs_inverse_kinematics
from dingo_control.Config import Configuration
from dingo_control.msg import TaskSpace, JointSpace, Angle
# Optional: if WebTuner is missing or fails to bind, the robot must still run.
try:
    from dingo_control.WebTuner import WebTuner
except Exception as _e:
    WebTuner = None
    rospy.logwarn("WebTuner unavailable (%s) - continuing without it", _e)

WEB_TUNER_PORT = 8080

STARTUP_MOVE_MS = 1800

if is_physical:
    from dingo_servo_interfacing.HardwareInterface_LX16A import (
        HardwareInterface, LOW_POWER_STAGGER_GROUPS, LOW_POWER_MOVE_TIME_MS)
    from dingo_servo_interfacing.HardwareInterface_Pico import WheelInterface
    from dingo_peripheral_interfacing.IMU import IMU
    from dingo_control.Config import Leg_linkage

LOW_POWER_RELAX_MASK = np.array([[0, 0, 0, 0],
                                 [1, 1, 1, 1],
                                 [1, 1, 1, 1]])

class DingoDriver:
    def __init__(self,is_sim, is_physical, use_imu):
        self.message_rate = 50
        self.rate = rospy.Rate(self.message_rate)

        self.is_sim = is_sim
        self.is_physical = is_physical
        self.use_imu = use_imu

        self.joint_command_sub = rospy.Subscriber("/joint_space_cmd", JointSpace, self.run_joint_space_command)
        self.task_command_sub = rospy.Subscriber("/task_space_cmd", TaskSpace, self.run_task_space_command)
        self.external_commands_enabled = 0

        if self.is_sim:
            self.sim_command_topics = ["/dingo_controller/FR_theta1/command",
                    "/dingo_controller/FR_theta2/command",
                    "/dingo_controller/FR_theta3/command",
                    "/dingo_controller/FL_theta1/command",
                    "/dingo_controller/FL_theta2/command",
                    "/dingo_controller/FL_theta3/command",
                    "/dingo_controller/RR_theta1/command",
                    "/dingo_controller/RR_theta2/command",
                    "/dingo_controller/RR_theta3/command",
                    "/dingo_controller/RL_theta1/command",
                    "/dingo_controller/RL_theta2/command",
                    "/dingo_controller/RL_theta3/command"]

            self.sim_publisher_array = []
            for i in range(len(self.sim_command_topics)):
                self.sim_publisher_array.append(rospy.Publisher(self.sim_command_topics[i], Float64, queue_size = 0))

        # Create config
        self.config = Configuration()
        if is_physical:
            self.linkage = Leg_linkage(self.config)
            for attempt in range(10):
                try:
                    self.hardware_interface = HardwareInterface(self.linkage)
                    break
                except Exception as e:
                    rospy.logwarn("Servo bus not ready (attempt %d/10): %s", attempt + 1, e)
                    time.sleep(2)
            else:
                rospy.logfatal("Could not connect to servo bus after 10 attempts")
                sys.exit(1)
            self.wheel_interface = WheelInterface(self.config)
        if self.use_imu:
            self.imu = IMU()

        # Create controller and user input handles
        self.controller = Controller(
            self.config,
            four_legs_inverse_kinematics,
        )

        self.state = State()
        self.servos_relaxed = False
        self.ds4_led = DS4Led()

        if is_physical:
            rospy.on_shutdown(self.shutdown_relax)

        self.web_tuner = None
        if WebTuner is not None:
            try:
                self.web_tuner = WebTuner(
                    self.config, self.state,
                    self.hardware_interface if is_physical else None,
                    port=WEB_TUNER_PORT)
                if not self.web_tuner.start():
                    self.web_tuner = None
            except Exception as e:
                rospy.logwarn("WebTuner failed to start (%s) - continuing", e)
                self.web_tuner = None

        rospy.loginfo("Creating input listener...")
        self.input_interface = InputInterface(self.config)
        rospy.loginfo("Input listener successfully initialised... Robot will now receive commands via Joy messages")

        rospy.loginfo("Summary of current gait parameters:")
        rospy.loginfo("overlap time: %.2f", self.config.overlap_time)
        rospy.loginfo("swing time: %.2f", self.config.swing_time)
        rospy.loginfo("z clearance: %.2f", self.config.z_clearance)
        rospy.loginfo("back leg x shift: %.2f", self.config.rear_leg_x_shift)
        rospy.loginfo("front leg x shift: %.2f", self.config.front_leg_x_shift)
        rospy.loginfo("back leg z trim: %.3f", self.config.rear_leg_z_shift)
        rospy.loginfo("max stride: %.2f", self.config.max_stride)
        rospy.loginfo("rear lift boost: %.3f  (front %.3f / rear %.3f)",
                      self.config.rear_z_clearance_boost, self.config.z_clearance,
                      self.config.z_clearance + self.config.rear_z_clearance_boost)
        if self.is_physical:
            rospy.loginfo("low-power servo stagger: %s  (R3 toggles)",
                          "ON - legs alternate %s each cycle, so only %d of 4 "
                          "move at a time (softer gait, lower peak current)"
                          % (" / ".join(str(g) for g in LOW_POWER_STAGGER_GROUPS),
                             len(LOW_POWER_STAGGER_GROUPS[0]))
                          if self.hardware_interface.low_power_mode
                          else "OFF - all 4 legs commanded every cycle")

        
    
    def shutdown_relax(self):
        """Release torque on every servo and stop the wheels."""
        if getattr(self, "_shutdown_done", False):
            return
        self._shutdown_done = True
        try:
            rospy.loginfo("Shutting down: releasing servo torque")
        except Exception:
            pass
        try:
            if self.web_tuner is not None:
                self.web_tuner.stop()
        except Exception:
            pass
        try:
            self.wheel_interface.stop()
        except Exception:
            pass
        try:
            self.hardware_interface.relax_all_motors()
            self.servos_relaxed = True
        except Exception as e:
            # print rather than rospy.logwarn: the logging system may already
            # be down by the time a shutdown hook runs
            print("Could not relax servos on shutdown: %s" % e)

    def run(self):
        try:
            self._run()
        finally:
            # Covers the paths rospy.on_shutdown does not: SystemExit from the
            # SIGINT handler, and any exception escaping the control loop.
            if self.is_physical:
                self.shutdown_relax()

    def _run(self):
        while not rospy.is_shutdown():
            rospy.loginfo("Manual robot control active. Currently not accepting external commands")
            command = self.input_interface.get_command(self.state,self.message_rate)
            self.state.behavior_state = BehaviorState.REST
            self.controller.run(self.state, command)
            self.controller.publish_joint_space_command(self.state.joint_angles)
            self.controller.publish_task_space_command(self.state.rotated_foot_locations)
            if self.is_sim:
                    self.publish_joints_to_sim(self.state.joint_angles)
            if self.is_physical:
                self.hardware_interface.set_actuator_postions(
                    self.state.joint_angles, move_time_ms=STARTUP_MOVE_MS)
                self.wheel_interface.stop()
                rospy.loginfo("Moving to REST over %.1f s", STARTUP_MOVE_MS / 1000.0)
                rospy.sleep(STARTUP_MOVE_MS / 1000.0)
            while True:
                time.start = rospy.Time.now()

                command = self.input_interface.get_command(self.state,self.message_rate)
                if command.joystick_control_event == 1:
                    self.external_commands_enabled = 1
                    break

                if command.stagger_toggle_event == 1 and self.is_physical:
                    self.hardware_interface.low_power_mode = (
                        not self.hardware_interface.low_power_mode)
                    rospy.loginfo(
                        "Low-power servo stagger: %s",
                        "ON - legs alternate %s each cycle, so only %d of 4 move "
                        "at a time with a %d ms glide (softer gait, halves peak "
                        "current)"
                        % (" / ".join(str(g) for g in LOW_POWER_STAGGER_GROUPS),
                           len(LOW_POWER_STAGGER_GROUPS[0]), LOW_POWER_MOVE_TIME_MS)
                        if self.hardware_interface.low_power_mode
                        else "OFF - all 4 legs commanded every cycle")

                self.state.euler_orientation = (
                    self.imu.read_orientation() if self.use_imu else np.array([0, 0, 0])
                )
                [yaw,pitch,roll] = self.state.euler_orientation

                self.controller.run(self.state, command)
                self.ds4_led.show_state(self.state.behavior_state,
                                        dimmed=not self.state.imu_stabilization_active)

                if self.state.behavior_state in (BehaviorState.TROT, BehaviorState.REST, BehaviorState.WHEELED, BehaviorState.CRAWL, BehaviorState.PAW):
                    self.controller.publish_joint_space_command(self.state.joint_angles)
                    self.controller.publish_task_space_command(self.state.rotated_foot_locations)

                    if self.is_sim:
                        self.publish_joints_to_sim(self.state.joint_angles)
                    if self.is_physical:
                        in_low_power = (self.state.behavior_state == BehaviorState.WHEELED
                                        and self.state.low_power_active)
                        if self.servos_relaxed and not in_low_power:
                            self.hardware_interface.engage_all_motors()
                            self.hardware_interface.set_actuator_postions(
                                self.state.joint_angles, move_time_ms=1000)
                            self.servos_relaxed = False
                            rospy.loginfo("Servos re-engaged")
                        if in_low_power and not self.servos_relaxed:
                            if abs(self.state.height - self.config.low_power_height) < 0.001:
                                self.hardware_interface.relax_all_motors(LOW_POWER_RELAX_MASK)
                                self.servos_relaxed = True
                                rospy.loginfo("Leg servos relaxed (hips stay engaged) -- low-power wheeled driving active")
                            else:
                                self.hardware_interface.set_actuator_postions(self.state.joint_angles)
                        elif not self.servos_relaxed:
                            self.hardware_interface.set_actuator_postions(self.state.joint_angles)
                        wheels_commanded = abs(command.wheel_throttle) > 0.01 or (
                            self.state.behavior_state == BehaviorState.WHEELED
                            and abs(command.wheel_yaw_rate) > 0.01
                        )
                        if wheels_commanded:
                            self.wheel_interface.drive(command)
                        else:
                            self.wheel_interface.stop()
                    time.end = rospy.Time.now()
                else:
                    if self.is_sim:
                        self.publish_joints_to_sim(self.state.joint_angles)
                self.rate.sleep()

            rospy.loginfo("Manual Control deactivated. Now accepting external commands")
            self.ds4_led.show_external_mode()
            command = self.input_interface.get_command(self.state,self.message_rate)
            self.state.behavior_state = BehaviorState.REST
            self.state.low_power_active = False
            self.controller.run(self.state, command)
            self.controller.publish_joint_space_command(self.state.joint_angles)
            self.controller.publish_task_space_command(self.state.rotated_foot_locations)
            if self.is_sim:
                    self.publish_joints_to_sim(self.state.joint_angles)
            if self.is_physical:
                if self.servos_relaxed:
                    # Handoff can happen mid low-power wheeled driving; put
                    # torque back on before commanding the REST pose.
                    self.hardware_interface.engage_all_motors()
                    self.servos_relaxed = False
                    rospy.loginfo("Servos re-engaged")
                    self.hardware_interface.set_actuator_postions(self.state.joint_angles, move_time_ms=1000)
                else:
                    self.hardware_interface.set_actuator_postions(self.state.joint_angles)
                self.wheel_interface.stop()
            while True:
                command = self.input_interface.get_command(self.state,self.message_rate)
                if command.joystick_control_event == 1:
                    self.external_commands_enabled = 0
                    break
                self.rate.sleep()

    def run_task_space_command(self, msg):
        if self.external_commands_enabled == 1:
            foot_locations = np.zeros((3,4))
            j = 0
            for i in range(3):
                foot_locations[i] = [msg.FR_foot[j], msg.FL_foot[j], msg.RR_foot[j], msg.RL_foot[j]]
                j = j+1
            joint_angles = self.controller.inverse_kinematics(foot_locations, self.config)
            if self.is_sim:
                self.publish_joints_to_sim(joint_angles)
            if self.is_physical and not self.servos_relaxed:
                self.hardware_interface.set_actuator_postions(joint_angles)
        else:
            rospy.logerr("ERROR: Robot not accepting commands. Please deactivate manual control before sending control commands")

    def run_joint_space_command(self, msg):
        if self.external_commands_enabled == 1:
            joint_angles = np.zeros((3,4))
            j = 0
            for i in range(3):
                joint_angles[i] = [msg.FR_foot[j], msg.FL_foot[j], msg.RR_foot[j], msg.RL_foot[j]]
                j = j+1
            if self.is_sim:
                self.publish_joints_to_sim(joint_angles)
            if self.is_physical and not self.servos_relaxed:
                self.hardware_interface.set_actuator_postions(joint_angles)
        else:
            rospy.logerr("ERROR: Robot not accepting commands. Please deactivate manual control before sending control commands")
    
    def publish_joints_to_sim(self, joint_angles):
        rows, cols = joint_angles.shape
        i = 0
        for col in range(cols):
            for row in range(rows):
                self.sim_publisher_array[i].publish(joint_angles[row,col])
                i = i + 1


def signal_handler(sig, frame):
    sys.exit(0)

def main():
    """Main program
    """
    rospy.init_node("dingo_driver") 
    signal.signal(signal.SIGINT, signal_handler)
    dingo = DingoDriver(is_sim, is_physical, use_imu)
    dingo.run()
    
main()
