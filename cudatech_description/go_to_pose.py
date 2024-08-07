#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class GoToPose(Node):
    def __init__(self):
        super().__init__('go_to_pose')
        self.navigator = BasicNavigator()
        self.target_pose = self.create_target_pose()
        self.publish_pose()

    def create_target_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 1.0  # Set your target x position
        pose.pose.position.y = 2.0  # Set your target y position
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # Assuming no rotation
        return pose

    def publish_pose(self):
        self.navigator.goToPose(self.target_pose)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info('Distance remaining: {:.2f}'.format(feedback.distance_remaining))
        
        result = self.navigator.getResult()
        if result == BasicNavigator.NavigationResult.SUCCEEDED:
            self.get_logger().info('Reached target pose')
        else:
            self.get_logger().info('Failed to reach target pose')

def main(args=None):
    rclpy.init(args=args)
    node = GoToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
