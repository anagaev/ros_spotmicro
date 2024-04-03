import numpy as np

from .base_state import BaseState

class State(BaseState):
    def __init__(self,
                 horizontal_velocity: np.ndarray,
                 yaw_rate: float,
                 pitrch: float,
                 roll: float,
                 activation: float,
                 height: float,
                 length: float,
                 width: float
                 ):
        super().__init__(horizontal_velocity,
                         yaw_rate,
                         pitrch,
                         roll,
                         activation,
                         height)

        self.ticks = 0
        self.foot_locations = self.init_foot_locations(length, width, height)
        self.joint_angles = np.zeros((3, 4))
        self.quat_orientation = np.array([1, 0, 0, 0])

    @staticmethod
    def init_foot_locations(length: float, width: float, height: float):
        foot_locations = np.array([[-0.5 * length, height, 0.5 * width ],
                                    [0.5 * length, height, 0.5 * width ],
                                    [-0.5 * length, height, -0.5 * width ],
                                    [0.5 * length, height,-0.5 * width ]]).T
        return foot_locations
