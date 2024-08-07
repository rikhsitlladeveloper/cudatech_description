from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info(f"{self.nodeName} started")

        self.degree = pi / 180.0
        self.loop_rate = self.create_rate(30)

        self.angle = 0.

    def publish_states(self):
        joint_state = JointState()
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'axis'

        while rclpy.ok():
            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['left_wheel_joint']
            joint_state.position = [self.angle]

            odom_trans.header.stamp = now.to_msg()
            odom_trans.transform.translation.x = cos(self.angle) * 2
            odom_trans.transform.translation.y = sin(self.angle) * 2
            odom_trans.transform.translation.z = 0.7
            odom_trans.transform.rotation = euler_to_quaternion(0, self.angle + pi/2, self.angle + pi/2)

            self.joint_pub.publish(joint_state)
            self.broadcaster.sendTransform(odom_trans)

            self.angle += self.degree * 10
            self.loop_rate.sleep()

def main():
    rclpy.init()
    node = StatePublisher()
    node.publish_states()

if __name__ == '__main__':
    main()
