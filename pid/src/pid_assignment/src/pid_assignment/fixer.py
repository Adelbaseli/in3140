#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import SetJointProperties


class GazeboFix(Node):
    def __init__(self):
        super().__init__('gazebo_fix')

        self.client = self.create_client(
            SetJointProperties,
            '/gazebo/set_joint_properties'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_joint_properties...')

        self.fix_joint('joint_1')
        self.fix_joint('joint_3')

    def fix_joint(self, name):
        req = SetJointProperties.Request()
        req.joint_name = name
        req.ode_joint_config.hiStop = [0.0, 0.0, 0.0]
        req.ode_joint_config.loStop = [0.0, 0.0, 0.0]
        req.ode_joint_config.vel = [0.0, 0.0, 0.0]

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            self.get_logger().fatal(f"Gazebo could not fix '{name}'!")
        else:
            self.get_logger().info(f"Set '{name}' fixed")


def main():
    rclpy.init()
    node = GazeboFix()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
