import numpy as np
from transforms3d.euler import euler2mat

class SwingController:
    def __init__(self, config):
        self.config = config

    def raibert_touchdown_location(
        self, leg_index, command, stance_ticks=None
    ):
        if stance_ticks is None:
            stance_ticks = self.config.stance_ticks
        delta_p_2d = (
            self.config.alpha
            * stance_ticks
            * self.config.dt
            * command.horizontal_velocity
        )
        delta_p = np.array([delta_p_2d[0], delta_p_2d[1], 0])
        theta = (
            self.config.beta
            * stance_ticks
            * self.config.dt
            * command.yaw_rate
        )
        R = euler2mat(0, 0, theta)
        return R @ self.config.default_stance[:, leg_index] + delta_p


    def leg_clearance(self, leg_index):
        """Peak swing lift for one leg. Legs 2,3 (BR,BL) get the rear boost,
        which compensates for the body sagging at the heavier rear end -- see
        Configuration.rear_z_clearance_boost."""
        clearance = self.config.z_clearance
        if leg_index >= 2:
            clearance += self.config.rear_z_clearance_boost
        return clearance

    def swing_height(self, swing_phase, clearance=None, triangular=True):
        if clearance is None:
            clearance = self.config.z_clearance
        if triangular:
            if swing_phase < 0.5:
                swing_height_ = swing_phase / 0.5 * clearance
            else:
                swing_height_ = clearance * (1 - (swing_phase - 0.5) / 0.5)
        return swing_height_


    def next_foot_location(
        self,
        swing_prop,
        leg_index,
        state,
        command,
        swing_ticks=None,
        stance_ticks=None,
    ):
        if swing_ticks is None:
            swing_ticks = self.config.swing_ticks
        assert swing_prop >= 0 and swing_prop <= 1
        foot_location = state.foot_locations[:, leg_index]
        swing_height_ = self.swing_height(swing_prop, self.leg_clearance(leg_index))
        touchdown_location = self.raibert_touchdown_location(leg_index, command, stance_ticks)
        time_left = self.config.dt * swing_ticks * (1.0 - swing_prop)
        v = (touchdown_location - foot_location) / time_left * np.array([1, 1, 0])
        delta_foot_location = v * self.config.dt
        z_shift = self.config.rear_leg_z_shift if leg_index >= 2 else 0.0
        z_vector = np.array([0, 0, swing_height_ + command.height + z_shift])
        return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location
