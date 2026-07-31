from dingo_control.Gaits import GaitController
from dingo_control.StanceController import StanceController
from dingo_control.SwingLegController import SwingController
from dingo_utilities.Utilities import clipped_first_order_filter
from dingo_control.State import BehaviorState, State
from dingo_control.msg import TaskSpace, JointSpace, Angle

import numpy as np
from transforms3d.euler import euler2mat, quat2euler
from transforms3d.quaternions import qconjugate, quat2axangle
from transforms3d.axangles import axangle2mat
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from math import degrees


class _CrawlGaitConfig:
    """Adapts Configuration's crawl_* fields to the plain attribute names"""

    def __init__(self, config):
        self._config = config

    @property
    def phase_length(self):
        return self._config.crawl_phase_length

    @property
    def num_phases(self):
        return self._config.crawl_num_phases

    @property
    def phase_ticks(self):
        return self._config.crawl_phase_ticks

    @property
    def contact_phases(self):
        return self._config.crawl_contact_phases


SPEED_FACTORS = (0.1, 0.2, 0.3, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0)

GAIT_IDLE_SPEED = 0.02      # m/s
GAIT_IDLE_YAW = 0.10        # rad/s

REAR_BOOST_STEP = 0.005
REAR_BOOST_MIN = 0.0
REAR_BOOST_MAX = 0.06


def _step_speed_factor(current, direction):
    """Next cadence multiplier one rung up (direction=+1) or down (-1) the ladder."""
    i = min(range(len(SPEED_FACTORS)), key=lambda k: abs(SPEED_FACTORS[k] - current))
    return SPEED_FACTORS[max(0, min(len(SPEED_FACTORS) - 1, i + direction))]


class Controller:
    """Controller and planner object
    """

    def __init__(
        self,
        config,
        inverse_kinematics,
    ):
        self.config = config

                ################# ROS PUBLISHER FOR TASK SPACE GOALS ##############
        self.task_space_pub = rospy.Publisher('task_space_goals', TaskSpace, queue_size=10)
        self.joint_space_pub = rospy.Publisher('joint_space_goals', JointSpace, queue_size=10)

        self.smoothed_yaw = 0.0  # for REST mode only
        self.inverse_kinematics = inverse_kinematics

        self.contact_modes = np.zeros(4)
        self.gait_controller = GaitController(self.config)
        self.crawl_gait_controller = GaitController(_CrawlGaitConfig(self.config))
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)
        self.paw_start_tick = 0
        self._gait_ticks = 0

        self.trot_transition_mapping = {BehaviorState.REST: BehaviorState.TROT, BehaviorState.TROT: BehaviorState.REST, BehaviorState.HOP: BehaviorState.TROT, BehaviorState.FINISHHOP: BehaviorState.TROT, BehaviorState.WHEELED: BehaviorState.TROT, BehaviorState.CRAWL: BehaviorState.TROT, BehaviorState.PAW: BehaviorState.TROT}
        self.wheeled_transition_mapping = {BehaviorState.REST: BehaviorState.WHEELED, BehaviorState.WHEELED: BehaviorState.REST}
        self.crawl_transition_mapping = {BehaviorState.REST: BehaviorState.CRAWL, BehaviorState.CRAWL: BehaviorState.REST}
        # PAW is one-shot (completes and returns to REST on its own), so
        # only an entry mapping is needed -- no reverse toggle.
        self.paw_transition_mapping = {BehaviorState.REST: BehaviorState.PAW}
        self.activate_transition_mapping = {BehaviorState.DEACTIVATED: BehaviorState.REST, BehaviorState.REST: BehaviorState.DEACTIVATED}


    def gait_is_commanded(self, command):
        """True when the operator is actually asking the robot to move."""
        return (float(np.linalg.norm(command.horizontal_velocity)) > GAIT_IDLE_SPEED
                or abs(float(command.yaw_rate)) > GAIT_IDLE_YAW)


    def crawl_body_shift(self, state):
        """Body translation (x, y) that keeps the CoM inside the support triangle."""
        cfg = self.config
        frac = getattr(cfg, "crawl_com_shift", 0.0)
        if frac <= 0.0:
            return np.zeros(2)

        n = cfg.crawl_num_phases
        swing_ticks = cfg.crawl_swing_ticks
        if swing_ticks <= 0:
            return np.zeros(2)

        # Centroid of the three planted feet for each phase. contact_phases is
        # [leg][phase], 1 = planted.
        stance = cfg.default_stance
        contacts = cfg.crawl_contact_phases
        centroids = []
        for phase in range(n):
            planted = [leg for leg in range(4) if contacts[leg][phase] == 1]
            if not planted:
                centroids.append(np.zeros(2))
                continue
            centroids.append(np.array([stance[0, planted].mean(),
                                       stance[1, planted].mean()]))

        phase = int(self._gait_ticks // swing_ticks) % n
        prog = (self._gait_ticks % swing_ticks) / float(swing_ticks)  # 0..1 in phase
        lead = min(max(getattr(cfg, "crawl_com_lead", 0.0), 0.0), 0.9)

        target = centroids[phase]
        if lead > 0.0 and prog > (1.0 - lead):
            f = (prog - (1.0 - lead)) / lead
            f = f * f * (3.0 - 2.0 * f)  # smoothstep: no velocity step
            target = (1.0 - f) * centroids[phase] + f * centroids[(phase + 1) % n]
        return frac * target


    def step_gait(self, state, command, gait_controller=None, swing_ticks=None, stance_ticks=None):
        """Calculate the desired foot locations for the next timestep"""
        gait_controller = gait_controller or self.gait_controller
        swing_ticks = swing_ticks or self.config.swing_ticks
        stance_ticks = stance_ticks or self.config.stance_ticks
        # self._gait_ticks, not state.ticks: the gait clock stops while the
        # robot is standing still, so it resumes from the same phase.
        contact_modes = gait_controller.contacts(self._gait_ticks)
        new_foot_locations = np.zeros((3, 4))
        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            foot_location = state.foot_locations[:, leg_index]
            if contact_mode == 1:
                new_location = self.stance_controller.next_foot_location(leg_index, state, command)
            else:
                swing_proportion = (
                    gait_controller.subphase_ticks(self._gait_ticks) / swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion,
                    leg_index,
                    state,
                    command,
                    swing_ticks,
                    stance_ticks,
                )
            new_foot_locations[:, leg_index] = new_location
        return new_foot_locations, contact_modes


    def publish_task_space_command(self, rotated_foot_locations):

        task_space_message = TaskSpace()
        task_space_message.FR_foot = Point(rotated_foot_locations[0, 0] - self.config.LEG_ORIGINS[0, 0], rotated_foot_locations[1, 0] - self.config.LEG_ORIGINS[1, 0], rotated_foot_locations[2, 0] - self.config.LEG_ORIGINS[2, 0])
        task_space_message.FL_foot = Point(rotated_foot_locations[0, 1] - self.config.LEG_ORIGINS[0, 1], rotated_foot_locations[1, 1] - self.config.LEG_ORIGINS[1, 1], rotated_foot_locations[2, 1] - self.config.LEG_ORIGINS[2, 1])
        task_space_message.RR_foot = Point(rotated_foot_locations[0, 2] - self.config.LEG_ORIGINS[0, 2], rotated_foot_locations[1, 2] - self.config.LEG_ORIGINS[1, 2], rotated_foot_locations[2, 2] - self.config.LEG_ORIGINS[2, 2])
        task_space_message.RL_foot = Point(rotated_foot_locations[0, 3] - self.config.LEG_ORIGINS[0, 3], rotated_foot_locations[1, 3] - self.config.LEG_ORIGINS[1, 3], rotated_foot_locations[2, 3] - self.config.LEG_ORIGINS[2, 3])
        task_space_message.header = Header(stamp = rospy.Time.now())
        self.task_space_pub.publish(task_space_message)

    def publish_joint_space_command(self, angle_matrix):

        joint_space_message = JointSpace()
        joint_space_message.FR_foot = Angle(degrees(angle_matrix[0, 0]), degrees(angle_matrix[1, 0]), degrees(angle_matrix[2, 0]))
        joint_space_message.FL_foot = Angle(degrees(angle_matrix[0, 1]), degrees(angle_matrix[1, 1]), degrees(angle_matrix[2, 1]))
        joint_space_message.RR_foot = Angle(degrees(angle_matrix[0, 2]), degrees(angle_matrix[1, 2]), degrees(angle_matrix[2, 2]))
        joint_space_message.RL_foot = Angle(degrees(angle_matrix[0, 3]), degrees(angle_matrix[1, 3]), degrees(angle_matrix[2, 3]))
        joint_space_message.header = Header(stamp = rospy.Time.now())
        self.joint_space_pub.publish(joint_space_message)
    

    def run(self, state, command):
        """Steps the controller forward one timestep"""

        previous_state = state.behavior_state

        ########## Update operating state based on command ######
        if command.joystick_control_event:
            state.behavior_state = self.activate_transition_mapping[state.behavior_state]
        elif command.trot_event:
            if state.low_power_active:
                # Servos are relaxed in the crouch -- entering a gait from
                # here would be violent. Force an explicit L3 exit first.
                rospy.loginfo("Ignoring trot toggle: exit low-power wheeled first (L3)")
            else:
                state.behavior_state = self.trot_transition_mapping[state.behavior_state]
        elif command.wheeled_event:
            # Circle toggles pure wheeled driving; only reachable from REST
            state.behavior_state = self.wheeled_transition_mapping.get(
                state.behavior_state, state.behavior_state)
        elif command.crawl_event:
            # Square toggles the crawl (static walk) gait; only from REST
            state.behavior_state = self.crawl_transition_mapping.get(
                state.behavior_state, state.behavior_state)
        elif command.paw_event:
            state.behavior_state = self.paw_transition_mapping.get(
                state.behavior_state, state.behavior_state)

        if previous_state != state.behavior_state:
            rospy.loginfo("State changed from %s to %s", str(previous_state), str(state.behavior_state))
            if state.behavior_state == BehaviorState.PAW:
                self.paw_start_tick = state.ticks
            if previous_state == BehaviorState.WHEELED and state.low_power_active:
                # Leaving WHEELED always ends low-power mode; the driver
                # re-engages servo torque on seeing the flag drop.
                state.low_power_active = False
                rospy.loginfo("Low-power wheeled OFF (left WHEELED state)")

        # L3 toggles low-power wheeled driving: body ramps down to the
        # mechanically locking crouch, then the driver releases servo torque.
        if command.low_power_event and state.behavior_state == BehaviorState.WHEELED:
            state.low_power_active = not state.low_power_active
            rospy.loginfo("Low-power wheeled %s", "ON" if state.low_power_active else "OFF")

        # Triangle toggles IMU stabilization independently of the behavior state
        if command.imu_toggle_event:
            state.imu_stabilization_active = not state.imu_stabilization_active
            rospy.loginfo("IMU stabilization %s",
                          "ON" if state.imu_stabilization_active else "OFF")

        if command.speed_up_event:
            state.speed_factor = _step_speed_factor(state.speed_factor, +1)
            rospy.loginfo("Gait speed: %.1fx", state.speed_factor)
        elif command.speed_down_event:
            state.speed_factor = _step_speed_factor(state.speed_factor, -1)
            rospy.loginfo("Gait speed: %.1fx", state.speed_factor)

        if command.z_clear_up_event or command.z_clear_down_event:
            step = REAR_BOOST_STEP if command.z_clear_up_event else -REAR_BOOST_STEP
            self.config.rear_z_clearance_boost = round(
                min(REAR_BOOST_MAX, max(REAR_BOOST_MIN,
                    self.config.rear_z_clearance_boost + step)), 4)
            at_limit = ""
            if self.config.rear_z_clearance_boost in (REAR_BOOST_MIN, REAR_BOOST_MAX):
                at_limit = "  (at limit)"
            rospy.loginfo(
                "Rear lift boost: %+.3f m  ->  front %.3f / rear %.3f m at "
                "height %.3f%s | copy into Config.rear_z_clearance_boost",
                self.config.rear_z_clearance_boost, self.config.z_clearance,
                self.config.z_clearance + self.config.rear_z_clearance_boost,
                state.height, at_limit,
            )

        self.config.overlap_time = self.config.base_overlap_time / state.speed_factor
        self.config.swing_time = self.config.base_swing_time / state.speed_factor
        self.config.crawl_swing_time = self.config.base_crawl_swing_time / state.speed_factor

        if state.behavior_state == BehaviorState.CRAWL:
            _stance_s = self.config.crawl_stance_ticks * self.config.dt
        else:
            _stance_s = self.config.stance_ticks * self.config.dt
        if _stance_s > 0:
            _max_speed = self.config.max_stride / _stance_s
            _speed = float(np.linalg.norm(command.horizontal_velocity))
            if _speed > _max_speed:
                command.horizontal_velocity = (
                    command.horizontal_velocity * (_max_speed / _speed)
                )
                rospy.loginfo_throttle(
                    2.0,
                    "Speed capped %.2f -> %.2f m/s to keep stride within leg "
                    "reach (gait speed %.1fx makes stance %.2f s)",
                    _speed, _max_speed, state.speed_factor, _stance_s,
                )

        if state.behavior_state == BehaviorState.TROT:
            if self.gait_is_commanded(command):
                state.foot_locations, contact_modes = self.step_gait(
                    state,
                    command,
                )
                self._gait_ticks += 1
            else:
                state.foot_locations = (self.config.default_stance
                                        + np.array([0, 0, command.height])[:, np.newaxis])
                contact_modes = np.ones(4)
                self._gait_ticks = 0

            # Apply the desired body rotation
            rotated_foot_locations = (
                euler2mat(
                    command.roll, command.pitch, 0.0
                )
                @ state.foot_locations
            )

            # Construct foot rotation matrix to compensate for body tilt
            if state.imu_stabilization_active:
                yaw,pitch,roll = state.euler_orientation
                correction_factor = 0.8
                max_tilt = 0.4
                roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
                pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)
                rmat = euler2mat(roll_compensation, pitch_compensation, 0)

                rotated_foot_locations = rmat @ rotated_foot_locations

            state.joint_angles = self.inverse_kinematics(
                rotated_foot_locations, self.config
            )
            state.rotated_foot_locations = rotated_foot_locations

        elif state.behavior_state == BehaviorState.CRAWL:
            if self.gait_is_commanded(command):
                state.foot_locations, contact_modes = self.step_gait(
                    state,
                    command,
                    self.crawl_gait_controller,
                    self.config.crawl_swing_ticks,
                    self.config.crawl_stance_ticks,
                )
                self._gait_ticks += 1
            else:
                state.foot_locations = (self.config.default_stance
                                        + np.array([0, 0, command.height])[:, np.newaxis])
                contact_modes = np.ones(4)
                self._gait_ticks = 0

            rotated_foot_locations = (
                euler2mat(
                    command.roll, command.pitch, 0.0
                )
                @ state.foot_locations
            )

            _shift = self.crawl_body_shift(state)
            rotated_foot_locations[0, :] -= _shift[0]
            rotated_foot_locations[1, :] -= _shift[1]

            if state.imu_stabilization_active:
                yaw, pitch, roll = state.euler_orientation
                correction_factor = 0.8
                max_tilt = 0.4
                roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
                pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)
                rmat = euler2mat(roll_compensation, pitch_compensation, 0)

                rotated_foot_locations = rmat @ rotated_foot_locations

            state.joint_angles = self.inverse_kinematics(
                rotated_foot_locations, self.config
            )
            state.rotated_foot_locations = rotated_foot_locations

        elif state.behavior_state == BehaviorState.PAW:
            elapsed = state.ticks - self.paw_start_tick

            base_stance = (
                self.config.default_stance
                + np.array([0, 0, state.height])[:, np.newaxis]
            )
            paw_target = base_stance[:, PAW_LEG_INDEX] + np.array(
                [PAW_FORWARD_OFFSET, 0, PAW_LIFT_OFFSET]
            )

            if elapsed < PAW_LIFT_TICKS:
                t = elapsed / PAW_LIFT_TICKS
            elif elapsed < PAW_LIFT_TICKS + PAW_HOLD_TICKS:
                t = 1.0
            elif elapsed < PAW_TOTAL_TICKS:
                t = 1.0 - (elapsed - PAW_LIFT_TICKS - PAW_HOLD_TICKS) / PAW_LOWER_TICKS
            else:
                t = 0.0
            smooth_t = t * t * (3 - 2 * t)  # smoothstep easing

            state.foot_locations = base_stance.copy()
            state.foot_locations[:, PAW_LEG_INDEX] = (
                (1 - smooth_t) * base_stance[:, PAW_LEG_INDEX] + smooth_t * paw_target
            )

            rotated_foot_locations = (
                euler2mat(command.roll, command.pitch, 0.0)
                @ state.foot_locations
            )
            # Deliberately no IMU compensation -- this is a scripted pose,
            # not a stabilized stance.
            state.joint_angles = self.inverse_kinematics(
                rotated_foot_locations, self.config
            )
            state.rotated_foot_locations = rotated_foot_locations

            if elapsed >= PAW_TOTAL_TICKS:
                state.behavior_state = BehaviorState.REST
                rospy.loginfo("Paw complete, returning to REST")

        elif state.behavior_state == BehaviorState.WHEELED:
            if state.low_power_active:
                step = self.config.z_speed * self.config.dt
                if state.height < self.config.low_power_height:
                    command.height = min(self.config.low_power_height, state.height + step)
                else:
                    command.height = max(self.config.low_power_height, state.height - step)
                command.pitch = 0.0
                command.roll = 0.0
                yaw_proportion = 0.0
            else:
                yaw_proportion = command.yaw_rate / self.config.max_yaw_rate
            self.smoothed_yaw += (
                self.config.dt
                * clipped_first_order_filter(
                    self.smoothed_yaw,
                    yaw_proportion * -self.config.max_stance_yaw,
                    self.config.max_stance_yaw_rate,
                    self.config.yaw_time_constant,
                )
            )
            state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, command.height])[:, np.newaxis]
            )
            rotated_foot_locations = (
                euler2mat(
                    command.roll,
                    command.pitch,
                    self.smoothed_yaw,
                )
                @ state.foot_locations
            )

            if state.imu_stabilization_active and not state.low_power_active:
                yaw, pitch, roll = state.euler_orientation
                correction_factor = 0.8
                max_tilt = 0.4
                roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
                pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)
                rmat = euler2mat(roll_compensation, pitch_compensation, 0)
                # rmat, NOT rmat.T -- see the note in the TROT branch. The
                # transpose is positive feedback and drives the tilt further.
                rotated_foot_locations = rmat @ rotated_foot_locations

            state.joint_angles = self.inverse_kinematics(
                rotated_foot_locations, self.config
            )
            state.rotated_foot_locations = rotated_foot_locations

        elif state.behavior_state == BehaviorState.REST:
            yaw_proportion = command.yaw_rate / self.config.max_yaw_rate
            self.smoothed_yaw += (
                self.config.dt
                * clipped_first_order_filter(
                    self.smoothed_yaw,
                    yaw_proportion * -self.config.max_stance_yaw,
                    self.config.max_stance_yaw_rate,
                    self.config.yaw_time_constant,
                )
            )
            # Set the foot locations to the default stance plus the standard height
            state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, command.height])[:, np.newaxis]
            )
            # Apply the desired body rotation
            rotated_foot_locations = (
                euler2mat(
                    command.roll,
                    command.pitch,
                    self.smoothed_yaw,
                )
                @ state.foot_locations
            )

            # Construct foot rotation matrix to compensate for body tilt
            if state.imu_stabilization_active:
                rotated_foot_locations = self.stabilise_with_IMU(rotated_foot_locations,state.euler_orientation)

            state.joint_angles = self.inverse_kinematics(
                rotated_foot_locations, self.config
            )
            state.rotated_foot_locations = rotated_foot_locations

        state.ticks += 1
        state.pitch = command.pitch
        state.roll = command.roll
        state.height = command.height

    def set_pose_to_default(self, state):
        state.foot_locations = (
            self.config.default_stance
            + np.array([0, 0, self.config.default_z_ref])[:, np.newaxis]
        )
        print(state.foot_locations)
        state.joint_angles = self.inverse_kinematics(
            state.foot_locations, self.config
        )
        return state.joint_angles
    def stabilise_with_IMU(self,foot_locations,orientation):
        ''' Applies euler orientatin data of pitch roall and yaw to stabilise hte robt. Current only applying to pitch.'''
        yaw,pitch,roll = orientation
        # print('Yaw: ',np.round(np.degrees(yaw)),'Pitch: ',np.round(np.degrees(pitch)),'Roll: ',np.round(np.degrees(roll)))
        correction_factor = 0.5  # gentler than the 0.8 used while walking
        max_tilt = 0.4 #radians
        roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
        pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)
        rmat = euler2mat(roll_compensation, pitch_compensation, 0)

        rotated_foot_locations = rmat @ foot_locations
        return rotated_foot_locations
