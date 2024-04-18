import numpy as np
from transforms3d.euler import euler2mat, quat2euler

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

from .gait_scheduler import GaitScheduler
from .stance_controller import StanceController
from .swing_controller import SwingController
from .kynematic import Kynematic

from .state import State
from .command import Command


class MovementControllerNode(Node):
    """Controller and planner object
    """

    def __init__(self):
        super().__init__('MovementController')
        self.get_logger().info('Creating MovementController')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('d_length', rclpy.Parameter.Type.DOUBLE),
                ('d_width', rclpy.Parameter.Type.DOUBLE),
                ('d_height', rclpy.Parameter.Type.DOUBLE),
                ('da_legs_length', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('int_n_phases', rclpy.Parameter.Type.INTEGER),
                ('d_z_time_constant', rclpy.Parameter.Type.DOUBLE),
                ('d_dt', rclpy.Parameter.Type.DOUBLE),
                ('d_overlap_time', rclpy.Parameter.Type.DOUBLE),
                ('d_swing_time', rclpy.Parameter.Type.DOUBLE),
                ('d_swing_z_time_constant', rclpy.Parameter.Type.DOUBLE),
                ('d_z_clearance', rclpy.Parameter.Type.DOUBLE),
                ('d_alpha', rclpy.Parameter.Type.DOUBLE),
                ('d_beta', rclpy.Parameter.Type.DOUBLE),
                ('d_overlap_time', rclpy.Parameter.Type.DOUBLE),
                ('d_yaw_rate', rclpy.Parameter.Type.DOUBLE),
                ('d_pitch', rclpy.Parameter.Type.DOUBLE),
                ('d_roll', rclpy.Parameter.Type.DOUBLE),
                ('d_activation', rclpy.Parameter.Type.DOUBLE)
            ]
        )
        contact_phases = np.array([[1, 1, 1, 0],
                                   [1, 0, 1, 1],
                                   [1, 0, 1, 1],
                                   [1, 1, 1, 0]])
        args = [arg.value for arg in self.get_parameters(['d_length', 'd_width', 'da_legs_length'])]
        self.kinematic = Kynematic(*args)
        args = [arg.value for arg in self.get_parameters(['int_n_phases', 'd_dt', 'd_overlap_time', 'd_swing_time'])]
        self.gait_scheduler = GaitScheduler(*args, contact_phases)

        args = [arg.value for arg in self.get_parameters(['d_z_time_constant', 'd_dt'])]
        self.stance_controller = StanceController(*args)
        args = [arg.value for arg in self.get_parameters(['d_z_clearance',
                                                          'd_alpha',
                                                          'd_beta',
                                                          'd_dt',
                                                          'd_overlap_time',
                                                          'd_swing_time',
                                                          'd_length',
                                                          'd_width',
                                                          'd_height',
                                                          'da_legs_length'])]
        self.swing_controller = SwingController(*args)

        self.publisher_joints_angles = self.create_publisher(Float32MultiArray, '/joints_angles', 10)
        self.timer = self.create_timer(0.001, self.send_joints_angles)

        '''
        self.subscriber_direction_command = self.create_subscription(Point,
                                                                     '/direction_command',
                                                                     self.update_horizontal_velocity_and_height,
                                                                     10)
        '''
        horizontal_velocity = np.array([10.0, 0.0])
        args = [arg.value for arg in self.get_parameters(['d_yaw_rate',
                                                         'd_pitch',
                                                         'd_roll',
                                                         'd_activation',
                                                         'd_height'])]
        self.test_command = Command(horizontal_velocity, *args)
        
        args = [arg.value for arg in self.get_parameters(['d_yaw_rate',
                                                            'd_pitch',
                                                            'd_roll',
                                                            'd_activation',
                                                            'd_height',
                                                            'd_length',
                                                            'd_width',
                                                            'da_legs_length'])]
        self.test_state = State(horizontal_velocity, *args)


    def calculate_next_foot_pos(self, state, command):
        """Calculate the desired foot locations for the next timestep

        Returns
        -------
        Numpy array (3, 4)
            Matrix of new foot locations.
        """
        contact_modes = self.gait_scheduler.contacts(state.ticks)
        new_foot_locations = np.ones((3, 4))
        self.get_logger().info(f'Contacts ' + ','.join([str(v) for v in contact_modes]))
        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            if contact_mode == 1:
                new_location = self.stance_controller.next_foot_location(leg_index, state, command)
            else:
                swing_proportion = (
                    self.gait_scheduler.subphase_ticks(state.ticks) / self.swing_controller.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion,
                    leg_index,
                    state,
                    command
                )
            new_foot_locations[:3, leg_index] = new_location
            self.get_logger().info(f'Leg {leg_index} ' + ','.join([str(v) for v in new_location]))
        return new_foot_locations, contact_modes
    
    def update_horizontal_velocity_and_height(self, msg: Point):
        diraction = np.array(msg.data)
        self.test_command.horizontal_velocity = diraction[:2]
        #to do: update height

    def send_joints_angles(self):
        """Steps the controller forward one timestep

        Parameters
        ----------
        controller : Controller
            Robot controller object.
        """
        self.get_logger().info('Send angles')
        self.test_state.foot_locations, contact_modes = self.calculate_next_foot_pos(
            self.test_state,
            self.test_command,
        )
        # Apply the desired body rotation
        rotated_foot_locations = (
            euler2mat(
                self.test_command.roll, self.test_command.pitch, 0.0
            )
            @ self.test_state.foot_locations
        )

        # Construct foot rotation matrix to compensate for body tilt
        (roll, pitch, yaw) = quat2euler(self.test_state.quat_orientation)
        correction_factor = 0.8
        max_tilt = 0.4
        roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
        pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)
        rmat = euler2mat(roll_compensation, pitch_compensation, 0)

        rotated_foot_locations = rmat.T @ rotated_foot_locations
        self.test_state.joint_angles = self.kinematic.get_angles(
            rotated_foot_locations, [roll, pitch, yaw], np.zeros(3)
        )
        self.test_state.ticks += 1
        self.test_state.pitch = self.test_command.pitch
        self.test_state.roll = self.test_command.roll
        self.test_state.height = self.test_command.height

        msg = Float32MultiArray()
        msg.data = self.test_state.joint_angles.reshape(-1).tolist()
        #self.get_logger().info(','.join([str(v) for v in msg.data]))
        self.publisher_joints_angles.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MovementControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
