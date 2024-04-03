import numpy as np
from transforms3d.euler import euler2mat

class StanceController:
    def __init__(self,
                 z_time_constant: float,
                 dt: float):
        self.z_time_constant = z_time_constant
        self.dt = dt

    def position_delta(self, leg_index, state, command):
        """Calculate the difference between the next desired body location and the current body location
        
        Parameters
        ----------


        Returns
        -------
        (Numpy array (3), Numpy array (3, 3))
            (Position increment, rotation matrix increment)
        """
        y = state.foot_locations[1, leg_index]
        v_xz = np.array(
            [
                -command.horizontal_velocity[0],
                1.0
                / self.z_time_constant
                * (state.height - y),
                -command.horizontal_velocity[1]
            ]
        )
        delta_p = v_xz * self.dt
        delta_R = euler2mat(0, -command.yaw_rate * self.dt, 0)
        return (delta_p, delta_R)

    # TODO: put current foot location into state
    def next_foot_location(self, leg_index, state, command):
        foot_location = state.foot_locations[:3, leg_index]
        (delta_p, delta_R) = self.position_delta(leg_index, state, command)
        incremented_location = delta_R @ foot_location + delta_p
        return incremented_location
