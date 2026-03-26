#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner_ros2')
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/crustcrawler/controller/follow_joint_trajectory'
        )

    def inverse_kinematic(self, position):
        """Simple IK for CrustCrawler (3-DOF)"""
        x, y, z = position
        l1 = 11.0
        l2 = 22.3
        l3 = 17.1
        l4 = 8.0
        l3_mark = np.sqrt(l4**2 + l3**2 + np.sqrt(2)/2*l4*l3)

        s = z - l1
        r = np.sqrt(x**2 + y**2)
        d = ((x**2 + y**2 + s**2 - l2**2 - l3_mark**2) / (2*l2*l3_mark))

        theta1 = np.arctan2(y, x)
        theta3 = np.arctan2(-np.sqrt(1 - d**2), d)
        theta2 = np.arctan2(s, r) - np.arctan2(l3_mark*np.sin(theta3), l2 + l3_mark*np.cos(theta3))
        return np.array([theta1, -theta2 + np.pi/2, -theta3])

    def generate_circle_path(self, origin, radius, num_points):
        path = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = origin[0] + radius * math.cos(angle)
            y = origin[1] + radius * math.sin(angle)
            z = origin[2]  # keep height constant
            path.append(np.array([x, y, z]))
        return path

    def generate_trajectory(self, path):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3']

        # Go to default first
        time = 4.0
        point = JointTrajectoryPoint()
        point.positions = [0., 0., np.pi/2.]
        point.time_from_start = Duration(seconds=time).to_msg()
        goal_msg.trajectory.points.append(point)

        # Move to first point
        time += 4.0
        point = JointTrajectoryPoint()
        point.positions = self.inverse_kinematic(path[0]).tolist()
        point.time_from_start = Duration(seconds=time).to_msg()
        goal_msg.trajectory.points.append(point)

        # Compute trajectory timing
        path_dist = sum(np.linalg.norm(p1 - p0) for p0, p1 in zip(path[:-1], path[1:]))
        time_delta = path_dist / len(path) / 2.0  # adjust speed

        for p in path[1:]:
            time += time_delta
            point = JointTrajectoryPoint()
            point.positions = self.inverse_kinematic(p).tolist()
            point.time_from_start = Duration(seconds=time).to_msg()
            goal_msg.trajectory.points.append(point)

        # Return to default
        time += 4.0
        point = JointTrajectoryPoint()
        point.positions = [0., 0., np.pi/2.]
        point.time_from_start = Duration(seconds=time).to_msg()
        goal_msg.trajectory.points.append(point)

        return goal_msg

    def send_goal(self, origin=[30, 0, 10], radius=5, num_points=20):
        self.action_client.wait_for_server()
        path = self.generate_circle_path(np.array(origin), radius, num_points)
        goal_msg = self.generate_trajectory(path)

        self.get_logger().info('Sending trajectory goal...')
        send_future = self.action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Trajectory execution finished')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlanner()
    planner.send_goal(origin=[30, 0, 10], radius=5, num_points=20)
    rclpy.spin(planner)


if __name__ == '__main__':
    main()
