#!/usr/bin/env python3

from __future__ import print_function
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import numpy as np
import rospy


def path_length(path):
    length = 0.0
    for p1, p0 in zip(path[1:], path):
        length += np.linalg.norm(p1 - p0)
    return length


def inverse_kinematic(position):
    x, y, z = position
    l1 = 11.0
    l2 = 22.3
    l3 = 17.1
    l4 = 8.0
    l3_mark = np.sqrt(l4 ** 2 + l3 ** 2 + np.sqrt(2) / 2.0 * l4 * l3)

    s = z - l1
    r = np.sqrt(x ** 2 + y ** 2)
    d = ((x ** 2 + y ** 2 + s ** 2 - l2 ** 2 - l3_mark ** 2)
         / (2. * l2 * l3_mark))

    theta1 = np.arctan2(y, x)
    theta3 = np.arctan2(-np.sqrt(1. - d ** 2), d)
    theta2 = np.arctan2(s, r) - np.arctan2(l3_mark * np.sin(theta3),
                                           l2 + l3_mark * np.cos(theta3))
    return np.array([theta1, theta2 * -1. + np.pi / 2., theta3 * -1.])


def create_trajectory_point(position, seconds):
    point = JointTrajectoryPoint()
    point.positions.extend(position)
    point.time_from_start = rospy.Duration(seconds)
    return point


def generate_path(origin, radius, num):
    """
    Generate circle points in 3D
    """
    path = []

    # Angle step between points
    angle_step = 2 * np.pi / num

    for i in range(num):
        angle = i * angle_step

        x = origin[0] + radius * np.cos(angle)
        y = origin[1] + radius * np.sin(angle)
        z = origin[2]  # constant height

        point = np.array([x, y, z])
        path.append(point)

    return path


def generate_movement(path):
    movement = FollowJointTrajectoryGoal()

    # Joint names
    movement.trajectory.joint_names.extend(['joint_1', 'joint_2', 'joint_3'])

    # Tolerances
    movement.goal_tolerance.extend([
        JointTolerance('joint_1', 0.1, 0., 0.),
        JointTolerance('joint_2', 0.1, 0., 0.),
        JointTolerance('joint_3', 0.1, 0., 0.)
    ])

    movement.goal_time_tolerance = rospy.Duration(0.5)

    # Move to default position first
    time = 4.0
    movement.trajectory.points.append(
        create_trajectory_point([0., 0., np.pi / 2.], time))

    # Move to first circle point
    time += 4.0
    movement.trajectory.points.append(
        create_trajectory_point(inverse_kinematic(path[0]), time))

    # Calculate speed timing
    length = path_length(path)
    time_delta = (length / 2.) / len(path)

    # Add rest of path points
    for point in path[1:]:
        time += time_delta
        movement.trajectory.points.append(
            create_trajectory_point(inverse_kinematic(point), time))

    # Return to default position
    time += 4.0
    movement.trajectory.points.append(
        create_trajectory_point([0., 0., np.pi / 2.], time))

    return movement


def draw_circle(origin, radius, num):
    client = actionlib.SimpleActionClient(
        '/crustcrawler/controller/follow_joint_trajectory',
        FollowJointTrajectoryAction)

    # Generate path
    path = generate_path(origin, radius, num)

    # Generate movement
    goal = generate_movement(path)

    # Wait for server
    client.wait_for_server()

    # Send goal
    client.send_goal(goal)

    # Wait for result
    client.wait_for_result()

    result = client.get_result()

    if not result.error_code:
        print("Crustcrawler done!")
    else:
        print("Crustcrawler failed due to: '{!s}'({!s})"
              .format(result.error_string, result.error_code))

    return result.error_code


if __name__ == '__main__':
    import argparse
    import sys

    parser = argparse.ArgumentParser(
        description="CrustCrawler circle drawer")

    parser.add_argument(
        '--origin', '-o', type=float, nargs=3,
        metavar=('x', 'y', 'z'), required=True)

    parser.add_argument(
        '--radius', '-r', type=float, default=5.0)

    parser.add_argument(
        '--num_points', '-n', type=int, default=4)

    args = parser.parse_args()

    args.origin = np.array(args.origin)

    if 3 >= args.num_points <= 101:
        sys.exit("Number of points must be in range [3, 101]")

    max_dist = np.linalg.norm(args.origin)
    if np.abs(max_dist - args.radius) < 20.0:
        sys.exit("Circle too close to robot or outside workspace!")

    rospy.init_node('circle_drawer', anonymous=True)

    try:
        sys.exit(draw_circle(args.origin, args.radius, args.num_points))
    except rospy.ROSInterruptException:
        sys.exit("Program aborted during circle drawing")
