#!/usr/bin/env python3

import rospy
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16

command_pub = rospy.Publisher('/gripstatus', Int16, queue_size=10)
joint_dict = {'joint0': 0.0, 'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0, 'joint5': 0.0}


def update_dict(data):
    global joint_dict
    joint_dict['joint5'] = data.position[-1]

def display_planned_path_callback(data, joint_state_publisher):
    # Create a dictionary with default values for all joints
    # joint_dict = {'joint_0': 0.0, 'joint_1': 0.0, 'joint_2': 0.0, 'joint_3': 0.0, 'joint_4': 0.0, 'joint_5': 0.0}
    global joint_dict
    # Check if there are any points in the trajectory
    if not data.trajectory[0].joint_trajectory.points:
        rospy.logwarn("No waypoints in the trajectory.")
    else:
        # Extract the last waypoint
        last_waypoint = data.trajectory[0].joint_trajectory.points[-1]

        # Extract joint positions and names
        joint_positions = last_waypoint.positions   
        joint_names = data.trajectory[0].joint_trajectory.joint_names # ["joint0", ""]

        if len(joint_positions) == 1:
            command_pub.publish(8)

        # Update the dictionary with the available joint positions
        for joint_name, position in zip(joint_names, joint_positions):
            if joint_name in joint_dict:
                joint_dict[joint_name] = position

        # Publish the JointState message
        joint_state_msg = JointState()
        joint_state_msg.name = list(joint_dict.keys())
        joint_state_msg.position = list(joint_dict.values()) 
        joint_state_publisher.publish(joint_state_msg)

        # Print or process the joint positions and names as needed
        rospy.loginfo("Last waypoint joint positions: %s", joint_positions)
        rospy.loginfo("Joint names and positions: %s", joint_dict)

def main():
    rospy.init_node('planned_path_listener', anonymous=True)

    # Create a publisher for the /command topic
    joint_state_publisher = rospy.Publisher('/joint_states/goal', JointState, queue_size=10)


    # Subscribe to the /move_group/display_planned_path topic
    rospy.Subscriber("/move_group/display_planned_path",
                     DisplayTrajectory,
                     display_planned_path_callback,
                     callback_args=joint_state_publisher)
    
    rospy.Subscriber('/joint_states/goal', JointState, update_dict)

    rospy.spin()

if __name__ == '__main__':
    main()