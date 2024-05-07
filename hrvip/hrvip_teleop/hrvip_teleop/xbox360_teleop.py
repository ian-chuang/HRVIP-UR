import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class Xbox360Teleop(Node):

    def __init__(self):
        super().__init__('xbox360_teleop')

        self.declare_parameter('max_gripper_size', 0.93)
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('twist_stamped_topic', 'cmd_vel')
        self.declare_parameter('gripper_command_topic', 'gripper_forward_position_controller/commands')
        self.declare_parameter('linear_scale', 0.1)
        self.declare_parameter('angular_scale', 0.1)

        self.max_gripper_size = self.get_parameter('max_gripper_size').value
        self.joy_topic = self.get_parameter('joy_topic').value
        self.twist_stamped_topic = self.get_parameter('twist_stamped_topic').value
        self.gripper_command_topic = self.get_parameter('gripper_command_topic').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value

        self.twist_publisher = self.create_publisher(TwistStamped, self.twist_stamped_topic, 1)
        self.gripper_publisher = self.create_publisher(Float64MultiArray, self.gripper_command_topic, 1)
        self.joy_subscription = self.create_subscription(
            Joy,
            self.joy_topic,
            self.topic_callback,
            1,
        )

    def topic_callback(self, msg: Joy) -> None:
        msg_copy = msg  # Make a copy to avoid modifying original message

        # Deadzone for axes
        for i in range(len(msg_copy.axes)):
            if abs(msg_copy.axes[i]) < 0.2:
                msg_copy.axes[i] = 0.0

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = msg_copy.axes[1] * self.linear_scale
        twist.twist.linear.y = msg_copy.axes[0] * self.linear_scale

        # Handle vertical movement
        if msg_copy.buttons[5]:
            twist.twist.linear.z = self.linear_scale
        elif msg_copy.buttons[4]:
            twist.twist.linear.z = -self.linear_scale
        else:
            twist.twist.linear.z = 0.0

        twist.twist.angular.x = -msg_copy.axes[3] * self.angular_scale
        twist.twist.angular.y = msg_copy.axes[4] * self.angular_scale

        # Handle rotation
        if msg_copy.buttons[3]:
            twist.twist.angular.z = self.angular_scale
        elif msg_copy.buttons[2]:
            twist.twist.angular.z = -self.angular_scale
        else:
            twist.twist.angular.z = 0.0

        self.twist_publisher.publish(twist)

        # Handle gripper control
        if msg_copy.buttons[0]:
            gripper_msg = Float64MultiArray()
            gripper_msg.data = [self.max_gripper_size]
            self.gripper_publisher.publish(gripper_msg)
        elif msg_copy.buttons[1]:
            gripper_msg = Float64MultiArray()
            gripper_msg.data = [0.0]
            self.gripper_publisher.publish(gripper_msg)


def main():
    rclpy.init()
    node = Xbox360Teleop()
    rclpy.spin(node)
    rclpy.shutdown()