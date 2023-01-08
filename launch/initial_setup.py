import os
import json
import rclpy
from geometry_msgs.msg import PoseStamped

from behaviours.navigator import BasicNavigator


CONFIG = json.loads(os.environ['CONFIG'])
NAME = os.environ['ROBOT_NAME']
NAMESPACE = os.environ['ROBOT_NAMESPACE']
POSE = json.loads(os.environ['ROBOT_POSE'])

rclpy.init()
nav = BasicNavigator()
initial_pose = PoseStamped()
initial_pose.header.frame_id = 'map'
initial_pose.header.stamp = nav.get_clock().now().to_msg()
initial_pose.pose.position.x = POSE['x']
initial_pose.pose.position.y = POSE['y']
initial_pose.pose.position.z = POSE['z']
initial_pose.pose.orientation.x = POSE['x_rot']
initial_pose.pose.orientation.y = POSE['y_rot']
initial_pose.pose.orientation.z = POSE['z_rot']
initial_pose.pose.orientation.w = POSE['w_rot']
nav.setInitialPose(initial_pose)
