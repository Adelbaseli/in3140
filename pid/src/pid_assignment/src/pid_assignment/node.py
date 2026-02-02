#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from .pid import PID


class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Declare parameters (replacement for dynamic_reconfigure)
        self.declare_parameter('p', 0.0)
        self.declare_parameter('i', 0.0)
        self.declare_parameter('d', 0.0)
        self.declare_parameter('c', 0.0)

        # PID controller
        self._pid = PID(logger=self.get_logger())

        # Internal state
        self._position = 0.0
        self._velocity = 0.0
        self._setpoint = 0.0
        self._effort = 0.0

        # Publishers
        self._out_eff = self.create_publisher(
            Float64,
            '/crustcrawler/joint2_controller/command',
            5
        )

        self._out_state = self.create_publisher(
            JointControllerState,
            'state',
            5
        )

        # Subscribers
        self.create_subscription(
            JointState,
            '/crustcrawler/joint_states',
            self.joint_states_callback,
            10
        )

        self.create_subscription(
            Float64,
            '/pid_controller/setpoint',
            self.setpoint_callback,
            10
        )

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Timer (30 Hz)
        self._last_time = self.get_clock().now()
        self.create_timer(1.0 / 30.0, self.update)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'p':
                self._pid.p = param.value
            elif param.name == 'i':
                self._pid.i = param.value
            elif param.name == 'd':
                self._pid.d = param.value
            elif param.name == 'c':
                self._pid.c = param.value

        return rclpy.parameter.SetParametersResult(successful=True)

    def joint_states_callback(self, states):
        try:
            index = states.name.index('joint_2')
            self._position = states.position[index]
            self._velocity = states.velocity[index]
        except ValueError:
            self.get_logger().warn("Could not find 'joint_2' in joint_states")

    def setpoint_callback(self, msg):
        self._setpoint = msg.data

    def update(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        if dt <= 0.0:
            return

        self._publish_effort(dt)
        self._publish_state(dt)

    def _publish_effort(self, dt):
        self._effort = self._pid(
            self._setpoint,
            self._position,
            self._velocity,
            dt
        )

        msg = Float64()
        msg.data = self._effort
        self._out_eff.publish(msg)

    def _publish_state(self, dt):
        msg = JointControllerState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command = self._effort
        msg.set_point = self._setpoint
        msg.process_value = self._position
        msg.process_value_dot = self._velocity
        msg.error = self._pid.error
        msg.time_step = dt
        msg.p = self._pid.p
        msg.i = self._pid.i
        msg.d = self._pid.d

        self._out_state.publish(msg)


def main():
    rclpy.init()
    node = PIDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
