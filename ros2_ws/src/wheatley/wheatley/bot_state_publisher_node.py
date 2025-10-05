from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


class StatePublisher(Node):
    def __init__(self):
        super().__init__("state_publisher")

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.broadcaster = TransformBroadcaster(self)

        # robot state
        self.degree = pi / 180.0
        self.tilt = 0.0
        self.tinc = self.degree
        self.swivel = 0.0
        self.angle = 0.0
        self.height = 0.0
        self.hinc = 0.005

        # message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = "odom"
        self.odom_trans.child_frame_id = "axis"

        # Run at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        # update joint_state
        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.name = ["swivel", "tilt", "periscope"]
        joint_state.position = [self.swivel, self.tilt, self.height]

        # update transform
        self.odom_trans.header.stamp = now
        self.odom_trans.transform.translation.x = cos(self.angle) * 2
        self.odom_trans.transform.translation.y = sin(self.angle) * 2
        self.odom_trans.transform.translation.z = 0.7
        self.odom_trans.transform.rotation = euler_to_quaternion(
            0, 0, self.angle + pi / 2
        )

        # publish
        self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(self.odom_trans)

        # update state
        self.tilt += self.tinc
        if self.tilt < -0.5 or self.tilt > 0.0:
            self.tinc *= -1
        self.height += self.hinc
        if self.height > 0.2 or self.height < 0.0:
            self.hinc *= -1
        self.swivel += self.degree
        self.angle += self.degree / 4


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(
        pitch / 2
    ) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(
        pitch / 2
    ) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(
        pitch / 2
    ) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(
        pitch / 2
    ) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
