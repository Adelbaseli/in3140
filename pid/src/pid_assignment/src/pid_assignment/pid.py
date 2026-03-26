#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState
import math

class PID:
    """General PID class with P, PD, PID, PIDD options."""
    def __init__(self, logger=None):
        self.p = 10.0
        self.i = 0.0
        self.d = 5.0
        self.c = 0.0
        self.error = 0.0
        self.integral = 0.0
        self._logger = logger

    def __call__(self, desired, current, velocity=None, dt=None, mode='PD'):
        if mode == 'P':
            return self.P_ctrl(desired, current, dt)
        elif mode == 'PD':
            return self.PD_ctrl(desired, current, velocity, dt)
        elif mode == 'PID':
            return self.PID_ctrl(desired, current, velocity, dt)
        elif mode == 'PIDD':
            return self.PIDD_ctrl(desired, current, velocity, dt)
        else:
            return 0.0

    def P_ctrl(self, desired, current, dt):
        self.error = desired - current
        effort = self.p * self.error
        self._log(effort)
        return effort

    def PD_ctrl(self, desired, current, velocity, dt):
        self.error = desired - current
        effort = self.p * self.error - self.d * (velocity or 0.0)
        self._log(effort)
        return effort

    def PID_ctrl(self, desired, current, velocity, dt):
        self.error = desired - current
        self.integral += self.error * (dt or 0.01)
        effort = self.p * self.error + self.i * self.integral - self.d * (velocity or 0.0)
        self._log(effort)
        return effort

    def PIDD_ctrl(self, desired, current, velocity, dt):
        self.error = desired - current
        self.integral += self.error * (dt or 0.01)
        nonlinear = self.c * math.tanh(self.error)
        effort = self.p * self.error + self.i * self.integral - self.d * (velocity or 0.0) + nonlinear
        self._log(effort)
        return effort

    def _log(self, effort):
        if self._logger:
            self._logger.debug(f"PID effort: {effort}")
        else:
            print(f"PID effort: {effort}")


class MultiJointPIDNode(Node):
    """ROS2 Node controlling all Crustcrawler joints with PID, with fixed joints support."""
    def __init__(self, joints=None, mode='P', fixed_joints=None):
        super().__init__('multi_joint_pid')
        self.joints = joints or [
            'joint1', 'joint2', 'joint3'
        ]
        self.mode = mode

        # Joints to freeze
        self.fixed_joints = fixed_joints or ['joint1', 'joint3']

        # Initialize PID controllers
        self.pid_controllers = {j: PID(self.get_logger()) for j in self.joints}

        # State storage
        self.current_positions = {j: 0.0 for j in self.joints}
        self.current_velocities = {j: 0.0 for j in self.joints}
        self.setpoints = {j: 0.0 for j in self.joints}

        # Publishers
        self.publishers = {
            j: self.create_publisher(Float64, f'/crustcrawler/{j}_controller/command', 10)
            for j in self.joints
        }
        self.state_publishers = {
            j: self.create_publisher(JointControllerState, f'/pid_controller/{j}_state', 10)
            for j in self.joints
        }

        # Subscribers
        self.create_subscription(JointState, '/crustcrawler/joint_states', self.joint_state_callback, 10)
        for j in self.joints:
            self.create_subscription(
                Float64,
                f'/pid_controller/{j}_setpoint',
                lambda msg, joint=j: self.setpoint_callback(msg, joint),
                10
            )

        # Timer (30 Hz)
        self._last_time = self.get_clock().now()
        self.create_timer(1.0 / 30.0, self.update)

    def joint_state_callback(self, msg):
        for j in self.joints:
            if j in msg.name:
                index = msg.name.index(j)
                self.current_positions[j] = msg.position[index]
                self.current_velocities[j] = msg.velocity[index]

    def setpoint_callback(self, msg, joint):
        # Ignore setpoints for fixed joints
        if joint not in self.fixed_joints:
            self.setpoints[joint] = msg.data

    def update(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self._last_time = now

        for j in self.joints:
            # If joint is fixed, force desired position to 0
            desired = 0.0 if j in self.fixed_joints else self.setpoints[j]

            effort = self.pid_controllers[j](
                desired,
                self.current_positions[j],
                self.current_velocities[j],
                dt,
                mode=self.mode
            )

            # Publish effort
            msg = Float64()
            msg.data = effort
            self.publishers[j].publish(msg)

            # Publish controller state
            state_msg = JointControllerState()
            state_msg.header.stamp = self.get_clock().now().to_msg()
            state_msg.set_point = desired
            state_msg.process_value = self.current_positions[j]
            state_msg.process_value_dot = self.current_velocities[j]
            state_msg.command = effort
            state_msg.error = self.pid_controllers[j].error
            state_msg.p = self.pid_controllers[j].p
            state_msg.i = self.pid_controllers[j].i
            state_msg.d = self.pid_controllers[j].d
            self.state_publishers[j].publish(state_msg)


def main():
    rclpy.init()
    node = MultiJointPIDNode(mode='PD', fixed_joints=['joint1', 'joint3'])
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
