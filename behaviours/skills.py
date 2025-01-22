import asyncio
import os
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import random

from .navigator import BasicNavigator, TaskResult
from .conversion import quaternion_from_euler

CONFIG = json.loads(os.environ['CONFIG'])
NAME = os.environ['ROBOT_NAME']
NAMESPACE = os.environ['ROBOT_NAMESPACE']
POSE = json.loads(os.environ['ROBOT_POSE'])


def formatlog(severity, who, loginfo, variable=None, skill=None, params=None):
    return f"[{severity}], {who}, {loginfo}, {variable}, {skill}, {params}"


class Skills(Node):
    def __init__(self, name: str = f"skills_{NAME}", wait_period: float = 1.0):
        super(Skills, self).__init__(name)
        self.nav_controller = BasicNavigator()
        self.initial_pose = PoseStamped()
        self.log_pub = self.create_publisher(String, '/log', 10)
        self.wait_period = wait_period

        self.sendmsg_pub = {}
        self.timer = {}
        self.waitmsg_sub = {}
        self.messages = {}

        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.nav_controller.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = POSE['x']
        self.initial_pose.pose.position.y = POSE['y']
        self.initial_pose.pose.position.z = POSE['z']
        self.initial_pose.pose.orientation.x = POSE['x_rot']
        self.initial_pose.pose.orientation.y = POSE['y_rot']
        self.initial_pose.pose.orientation.z = POSE['z_rot']
        self.initial_pose.pose.orientation.w = POSE['w_rot']

        self.nav_controller.setInitialPose(self.initial_pose)
        self.nav_controller.waitUntilNav2Active()

    def poselist_to_posestamped(self, pose):
        quaternion = quaternion_from_euler(0, 0, pose[-1])

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav_controller.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose[0]
        goal_pose.pose.position.y = pose[1]
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]
        return goal_pose

    def publish_log(self, message: str, severity: str = "INFO", variable=None, skills=None, params=None):
        log_msg = String()
        log_msg.data = formatlog(severity, NAME, message, variable, skills, params)
        self.log_pub.publish(log_msg)

    async def navigate(self, goal_poses: [PoseStamped]):
        self.nav_controller.goThroughPoses(goal_poses)
        while not self.nav_controller.isTaskComplete():
            await asyncio.sleep(1)
        return self.nav_controller.getResult() == TaskResult.SUCCEEDED

    async def create_wait_message(self, topic: str):
        self.waitmsg_sub[topic] = self.create_subscription(String, topic, (lambda msg: self.msg_received(msg, topic)), 1)
        self.messages[topic] = None

    def msg_received(self, msg, topic):
        self.publish_log("message received")
        self.messages[topic] = msg.data

    def msg_not_received(self, topic):
        self.publish_log("message not received")
        self.messages[topic] = False

    async def wait_for_message(self, topic: str, wait_period: float = None):
        # Blocks until message is received or time limit expires
        wait_period = wait_period if wait_period is not None else self.wait_period
        self.timer = self.create_timer(wait_period, (lambda _: self.msg_not_received(topic)))

        while self.messages[topic] is None:
            rclpy.spin_once(self)

        self.timer.destroy()
        message = self.messages[topic] or None
        self.messages[topic] = None
        return message

    async def send_message(self, topic: str, message: str):
        try:
            self.sendmsg_pub[topic]
        except KeyError:
            self.sendmsg_pub[topic] = self.create_publisher(String, topic, 1)
        except TypeError:
            pass
        if random.random() < 0.03:
            return False

        self.sendmsg_pub[topic].publish(String(data=message))
        self.publish_log("sent message")
        return True

    async def approach_person(self, goal_pose: PoseStamped):
        self.nav_controller.goToPose(goal_pose)

    async def approach_robot(self, goal_pose: PoseStamped):
        self.nav_controller.goToPose(goal_pose)

    async def authenticate_person(self):
        return await self.send_message("/led_strip/display", message='nurse')

    async def operate_drawer(self):
        return await self.send_message('/drawer', message='operate')

    async def approach(self, target):
        return await self.send_message(f"/approach_{target}", message="performed approach")


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

    goal_poses = []
    for pose in path:
        goal_poses.append(skills.poselist_to_posestamped(pose))
    # asyncio.run(skills.navigate(goal_poses))
    print(asyncio.run(skills.authenticate_person()))
    rclpy.spin(skills)
