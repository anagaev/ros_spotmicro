import numpy as np
from transforms3d.euler import euler2mat

from .state import State

class SwingController:
    def __init__(self,
                 z_clearance: float,
                 alpha: float,
                 beta: float,
                 dt: float,
                 overlap_time: float,
                 swing_time: float,
                 length: float,
                 width: float,
                 height: float,
                 legs_length: np.ndarray):
        self.z_clearance = z_clearance
        self.alpha = alpha
        self.beta = beta
        self.dt = dt

        overlap_time = overlap_time
        swing_time = swing_time

        overlap_ticks = overlap_time / self.dt
        self.swing_ticks = swing_time / self.dt

        self.stance_ticks = 2 * overlap_ticks + self.swing_ticks

        self.default_stance = State.init_foot_locations(length, width, height, legs_length[0])

    def raibert_touchdown_location(
        self, leg_index, command
    ):
        delta_p_2d = (
            self.alpha
            * self.stance_ticks
            * self.dt
            * command.horizontal_velocity
        )
        delta_p = np.array([delta_p_2d[0], 0, delta_p_2d[1]])
        theta = (
            self.beta
            * self.stance_ticks
            * self.dt
            * command.yaw_rate
        )
        R = euler2mat(0, theta, 0)
        return R @ self.default_stance[:, leg_index] + delta_p


    def swing_height(self, swing_phase, triangular=True):
        if triangular:
            if swing_phase < 0.5:
                swing_height_ = swing_phase / 0.5 * self.z_clearance
            else:
                swing_height_ = self.z_clearance * (1 - (swing_phase - 0.5) / 0.5)
        return swing_height_ 


    def next_foot_location(
        self,
        swing_prop,
        leg_index,
        state,
        command,
    ):
        assert swing_prop >= 0 and swing_prop <= 1
        foot_location = state.foot_locations[:3, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(leg_index, command)
        time_left = self.dt * self.swing_ticks * (1.0 - swing_prop)
        v = (touchdown_location - foot_location[:3]) / time_left * np.array([1, 0, 1])
        delta_foot_location = v * self.dt
        y_vector = np.array([0, swing_height_ + command.height, 0])
        return foot_location[:3] * np.array([1, 0, 1]) + y_vector + delta_foot_location
