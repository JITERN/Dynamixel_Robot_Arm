#!/usr/bin/env python

import rospy
from rospy import Rate
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

global joint_trajectory_msg
joint_trajectory_msg = JointTrajectory()

def jointStatesCallback(joint_states_msg):
    # Extract joint positions from the joint_states_msg
    joint_positions = joint_states_msg.position

    # Create a JointTrajectory message
    global joint_trajectory_msg
    joint_trajectory_msg = JointTrajectory()

    # Populate the joint names (modify based on your robot's joints)
    joint_trajectory_msg.joint_names = joint_states_msg.name[:2]

    # Create a JointTrajectoryPoint
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.positions = joint_positions[:2]
    # trajectory_point.time_from_start = rospy.Duration(0.1)  # Specify the time duration for the trajectory point

    # Add the trajectory point to the message
    # joint_trajectory_msg.points.append(trajectory_point)
    joint_trajectory_msg.points = [trajectory_point]


    # Publish the JointTrajectory message to the /joint_trajectory topic
    # joint_trajectory_publisher.publish(joint_trajectory_msg)

if __name__ == '__main__':
    rospy.init_node('moveit_to_dynamixel_interface')  # Initialize your custom node
    rospy.loginfo("MoveIt! to Dynamixel Interface Node Started")

    # Create a subscriber for the /joint_states topic published by MoveIt!
    rospy.Subscriber("/joint_states", JointState, jointStatesCallback)

    # Create a publisher for the /joint_trajectory topic
    joint_trajectory_publisher = rospy.Publisher('/dynamixel_workbench/joint_trajectory', JointTrajectory, queue_size=10)

    # Define the desired publishing rate (e.g., 1 Hz)
    rate = Rate(20)  # 10 Hz

    while not rospy.is_shutdown():
        # Your callback and publishing logic here
        joint_trajectory_publisher.publish(joint_trajectory_msg)
        # Sleep to control the publishing rate
        rate.sleep()
    # rospy.spin()