import asyncio
import os
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from navigator import BasicNavigator
from conversion import quaternion_from_euler

CONFIG = json.loads(os.environ['CONFIG'])
NAME = os.environ['ROBOT_NAME']
NAMESPACE = os.environ['ROBOT_NAMESPACE']
POSE = json.loads(os.environ['ROBOT_POSE'])


class Skills(Node):
    def __init__(self, name: str = f"skills_{NAME}", wait_period: float = 1.0):
        super(Skills, self).__init__(name)
        self.nav = BasicNavigator()
        self.initial_pose = PoseStamped()
        self.log_pub = self.create_publisher(String, '/log', 10)
        self.wait_period = wait_period
        self.waitmsg_sub = None
        self.sendmsg_pub = None
        self.timer = None

        self.message = None

        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = POSE['x']
        self.initial_pose.pose.position.y = POSE['y']
        self.initial_pose.pose.position.z = POSE['z']
        self.initial_pose.pose.orientation.x = POSE['x_rot']
        self.initial_pose.pose.orientation.y = POSE['y_rot']
        self.initial_pose.pose.orientation.z = POSE['z_rot']
        self.initial_pose.pose.orientation.w = POSE['w_rot']

        self.nav.setInitialPose(self.initial_pose)

    def publish_log(self, message: str):
        now = str(self.get_clock().now())
        log_msg = String()
        log_msg.data = f"{now}_{message}"
        self.log_pub.publish(log_msg)

    async def navigate(self, waypoints: [[float]]):
        goal_poses = []
        for waypoint in waypoints:
            quaternion = quaternion_from_euler(0, 0, waypoint[-1])

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
            goal_pose.pose.position.x = waypoint[0]
            goal_pose.pose.position.y = waypoint[1]
            goal_pose.pose.orientation.x = quaternion[0]
            goal_pose.pose.orientation.y = quaternion[1]
            goal_pose.pose.orientation.z = quaternion[2]
            goal_pose.pose.orientation.w = quaternion[3]

            goal_poses.append(goal_pose)

        self.nav.followWaypoints(goal_poses)

    async def create_wait_message(self, topic: str):
        self.waitmsg_sub = self.create_subscription(String, topic, self.msg_received, 1)

    def msg_received(self, msg):
        self.publish_log("message received")
        self.message = msg.data

    def msg_not_received(self):
        self.publish_log("message not received")
        self.message = False

    async def wait_message(self, wait_period: float = None):
        # Blocks until message is received or time limit expires
        wait_period = wait_period if wait_period is not None else self.wait_period
        self.timer = self.create_timer(wait_period, self.msg_not_received)

        while self.message is None:
            rclpy.spin_once(self)

        self.timer.destroy()
        message = self.message or None
        self.message = None
        return message

    async def send_message(self, topic: str, message: str):
        self.sendmsg_pub = self.create_publisher(String, topic, 1)
        self.sendmsg_pub.publish(String(data=f"{message}"))
        self.publish_log("sent message")

    async def approach_person(self):
        pass

    async def approach_robot(self):
        pass

    async def authenticate_person(self):
        await self.create_wait_message("nurse/fauth")
        await self.send_message("/led_strip/display", message='nurse')
        auth_msg = await self.wait_message()
        return auth_msg == 'auth'

    async def operate_drawer(self):
        await self.send_message('/drawer', message='operate')
        return True


if __name__ == "__main__":
    rclpy.init()
    skills = Skills()
    path = [
        [
            -27.23,
            18.0,
            -1.57
        ],
        [
            -27.23,
            16.0
        ],
        [
            -28.5,
            16.0
        ],
        [
            -28.5,
            18.0,
            -1.57
        ]
    ]

    assert asyncio.run(skills.authenticate_person())
    rclpy.spin(skills)
