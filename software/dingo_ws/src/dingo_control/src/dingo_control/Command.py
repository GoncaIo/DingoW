import numpy as np


class Command:
    """Stores movement command
    """

    def __init__(self):
        self.horizontal_velocity = np.array([0, 0])
        # Options/Share: live swing-lift (z_clearance) tuning aid
        self.z_clear_up_event = 0
        self.z_clear_down_event = 0
        # Touchpad: toggle LOW_POWER_MODE servo staggering
        self.stagger_toggle_event = 0
        self.yaw_rate = 0.0
        self.height = -0.23  # matches State.height
        self.pitch = 0.0
        self.roll = 0.0
        self.joystick_control_active = 0
        self.trotting_active = 0

        self.height_movement = 0
        self.roll_movement = 0
        self.wheel_throttle = 0.0  # -1..1; R2/L2 triggers, or right stick Y in WHEELED mode
        self.wheel_yaw_rate = 0.0  # rad/s steering for the wheels (right stick X)
        
        self.hop_event = False
        self.trot_event = False
        self.wheeled_event = False
        self.crawl_event = False
        self.imu_toggle_event = False
        self.joystick_control_event = False
        self.speed_up_event = False
        self.speed_down_event = False
        self.paw_event = False
        self.low_power_event = False