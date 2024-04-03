import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from evdev import InputDevice, categorize, ecodes


class DualsenseReceveirNode(Node):
    """Get commands from dualsense and publish command for spot.
    """
    def __init__(self):
        super().__init__('DualsenseReceveir')
        self.get_logger().info('Creating DualsenseReceveir')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('event_name', rclpy.Parameter.Type.STRING),
                ('center', rclpy.Parameter.Type.DOUBLE),
                ('blind_value', rclpy.Parameter.Type.INTEGER),
                ('button_codes.x', rclpy.Parameter.Type.INTEGER),
                ('button_codes.circle', rclpy.Parameter.Type.INTEGER),
                ('button_codes.triangle', rclpy.Parameter.Type.INTEGER),
                ('button_codes.square', rclpy.Parameter.Type.INTEGER),
                ('button_codes.L1', rclpy.Parameter.Type.INTEGER),
                ('button_codes.R1', rclpy.Parameter.Type.INTEGER),
                ('button_codes.L2', rclpy.Parameter.Type.INTEGER),
                ('button_codes.R2', rclpy.Parameter.Type.INTEGER),
                ('button_codes.create', rclpy.Parameter.Type.INTEGER),
                ('button_codes.options', rclpy.Parameter.Type.INTEGER),
                ('button_codes.playstation', rclpy.Parameter.Type.INTEGER),
                ('button_codes.L3', rclpy.Parameter.Type.INTEGER),
                ('button_codes.R3', rclpy.Parameter.Type.INTEGER),
                ('button_values.up', rclpy.Parameter.Type.INTEGER),
                ('button_values.down', rclpy.Parameter.Type.INTEGER),
                ('absolutes.left_joystick_lr', rclpy.Parameter.Type.INTEGER),
                ('absolutes.left_joystick_ud', rclpy.Parameter.Type.INTEGER),
                ('absolutes.right_joystick_lr', rclpy.Parameter.Type.INTEGER),
                ('absolutes.right_joystick_ud', rclpy.Parameter.Type.INTEGER),
                ('leftpad_lr_values.left', rclpy.Parameter.Type.INTEGER),
                ('leftpad_lr_values.stop', rclpy.Parameter.Type.INTEGER),
                ('leftpad_lr_values.right', rclpy.Parameter.Type.INTEGER),
                ('leftpad_ud_values.up', rclpy.Parameter.Type.INTEGER),
                ('leftpad_ud_values.stop', rclpy.Parameter.Type.INTEGER),
                ('leftpad_ud_values.down', rclpy.Parameter.Type.INTEGER),
            ]
        )

        self.publisher_diraction_command = self.create_publisher(Point, '/direction_command', 10)
        self.timer = self.create_timer(0.1, self.send_joints_angles)

        self.delta_height = 0.0
        self.delta_height_step = 0.05
        center = self.get_parameter('center').value
        self.left_joystick_position = [center, center]
        self.gamepad = InputDevice('/dev/input/' + self.get_parameter('event_name').value)
        self.absolutes = self.get_parameters_by_prefix('absolutes')
        self.leftpad_ud_values = self.get_parameters_by_prefix('leftpad_ud_values')

    def read_commands_from_dualsense(self, event):
        if event.type == ecodes.EV_ABS:
            # check left joystick
            if event.code == self.absolutes['left_joystick_lr']:
                self.left_joystick_position[0] = event.value
            elif event.code == self.absolutes['left_joystick_ud']:
                self.left_joystick_position[1] = event.value
            # check leftpad up and down
            if event.code == self.absolutes['leftpad_ud']:
                # up and down action for height
                self.delta_height += event.code * self.delta_height_step

    def send_new_command(self):
        event = next(self.gamepad.read_loop())
        self.read_commands_from_dualsense(event)
        msg = Point()
        msg.data = self.left_joystick_position + self.delta_height
        self.publisher_diraction_command.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DualsenseReceveirNode()
    rclpy.spin(node)
    rclpy.shutdown()
