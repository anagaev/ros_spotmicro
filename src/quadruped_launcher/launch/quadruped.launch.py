import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    movement_controller_config = os.path.join(
        get_package_share_directory('quadruped_launcher'),
        'configs',
        'movement_controller_config.yaml'
    )
    movement_controller_node = Node(
        package='movement_controller',
        executable='movement_controller',
        parameters=[movement_controller_config]

    )

    servo_controller_config = os.path.join(
        get_package_share_directory('quadruped_launcher'),
        'configs',
        'servo_controller_config.yaml'
    )
    servo_controller_node = Node(
        package='servo_controller',
        executable='servo_controller',
        parameters=[servo_controller_config]

    )
    '''
    dualsense_receiver_config = os.path.join(
        get_package_share_directory('quadruped_launcher'),
        'configs',
        'dualsense_receiver_config.yaml'
    )
    dualsense_receiver_node = Node(
        package='dualsense_receiver',
        executable='dualsense_receiver',
        parameters=[dualsense_receiver_config]
    )
    '''

    return LaunchDescription([
        servo_controller_node,
        movement_controller_node
    ])

