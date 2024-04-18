import numpy as np

from .base_state import BaseState

class Command(BaseState):
    """Stores movement command
    """

    def __init__(self,
                 horizontal_velocity: np.ndarray,
                 yaw_rate: float,
                 pitrch: float,
                 roll: float,
                 activation: float,
                 height: float,
                ):
        super().__init__(horizontal_velocity,
                         yaw_rate,
                         pitrch,
                         roll,
                         activation,
                         height
                        )
        self.horizontal_velocity = np.array([25.0, 0.0])       
        self.hop_event = False
        self.trot_event = False
        self.activate_event = False