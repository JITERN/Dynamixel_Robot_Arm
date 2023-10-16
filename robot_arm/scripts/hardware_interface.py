#!/usr/bin/env python

import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib

class MyRobotHWInterface:
    def __init__(self):
        rospy.init_node('hardware_interface')
        # rospy.init_node('my_robot_hw_interface_node')

        # Action client for FollowJointTrajectory
        # self.client = actionlib.SimpleActionClient('/joint_trajectory_controller/follow_joint_trajectory',
        #                                            FollowJointTrajectoryAction)
        self.client = actionlib.SimpleActionClient('/effort_joint_trajectory_controller/follow_joint_trajectory',
                                                   FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def execute_trajectory(self, joint_positions):
        # Create a FollowJointTrajectoryGoal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["joint1", "joint2", "joint3"]  # Replace with your actual joint names

        # Add a trajectory point
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = joint_positions
        trajectory_point.time_from_start = rospy.Duration(2.0)  # Replace with your desired duration

        goal.trajectory.points.append(trajectory_point)

        # Send the goal and wait for the result
        self.client.send_goal_and_wait(goal)

if __name__ == '__main__':
    try:
        my_robot_hw_interface = MyRobotHWInterface()
        joint_positions = [1.0, -0.5, 0.2]  # Replace with your desired joint positions
        my_robot_hw_interface.execute_trajectory(joint_positions)
    except rospy.ROSInterruptException:
        pass
