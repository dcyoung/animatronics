import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState


class StatePublisher(Node):
    def __init__(self):
        super().__init__("state_publisher")

        # Create publisher
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)

        # Joint names
        self.joint_names = ["arm_joint", "spinner_joint"]

        # Timer: update at 50 Hz
        self.timer = self.create_timer(0.02, self.timer_callback)

        # Phase for sine wave
        self.start_time = self.get_clock().now().nanoseconds / 1e9  # seconds

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.start_time

        # Oscillate between -180 and 180 degrees
        angle_rad = math.radians(180)  # amplitude
        freq_arm = 0.1  # Hz (slow)
        freq_spinner = 0.05  # Hz (even slower)

        arm_angle = angle_rad * math.sin(2 * math.pi * freq_arm * t)
        spinner_angle = angle_rad * math.sin(2 * math.pi * freq_spinner * t)

        # Publish joint state
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [arm_angle, spinner_angle]

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
