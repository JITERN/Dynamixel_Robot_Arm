#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState

class MoveItControllerBridge:
    def __init__(self):
        rospy.init_node('moveit_controller_bridge', anonymous=True)

        # Subscriber to MoveIt-generated joint trajectory commands
        rospy.Subscriber('/moveit/joint_trajectory/goal', Float64MultiArray, self.moveit_joint_trajectory_callback)

        # Publisher for current joint positions
        self.feedback_publisher = rospy.Publisher('/feedback', JointState, queue_size=1)

    def moveit_joint_trajectory_callback(self, msg):
        # Extract joint positions from the received message
        joint_positions = msg.data

        # Publish joint positions to your low-level controller (send commands to /goal)

    def publish_current_joint_positions(self):
        # This is where you get the current joint positions from your low-level controller
        current_joint_positions = [0,0,0]

        # Create a JointState message for MoveIt
        joint_state_msg = JointState()
        joint_state_msg.name = ["joint1", "joint2", "joint3"]  # Replace with your joint names
        joint_state_msg.position = current_joint_positions

        # Publish the joint state feedback to MoveIt
        self.feedback_publisher.publish(joint_state_msg)

    def run(self):
        rate = rospy.Rate(10)  # Adjust the rate according to your needs
        while not rospy.is_shutdown():
            self.publish_current_joint_positions()
            rate.sleep()

if __name__ == '__main__':
    controller_bridge = MoveItControllerBridge()
    controller_bridge.run()