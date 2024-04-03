import numpy as np


class BaseState:
    '''A parent class for State and Command.
    '''
    def __init__(self,
                 horizontal_velocity: np.ndarray,
                 yaw_rate: float,
                 pitrch: float,
                 roll: float,
                 activation: float,
                 height: float):
        self.horizontal_velocity = horizontal_velocity
        self.yaw_rate = yaw_rate
        self.height = height
        self.pitch = pitrch
        self.roll = roll
        self.activation = activation