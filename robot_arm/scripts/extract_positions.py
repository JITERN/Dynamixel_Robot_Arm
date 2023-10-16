import rospy
from moveit_msgs.msg import MotionPlanResponse

def extract_joint_trajectory(response_msg):
    # Check if the message has the expected fields
    if hasattr(response_msg, 'trajectory') and hasattr(response_msg.trajectory, 'joint_trajectory'):
        # Access the joint_trajectory field
        joint_trajectory = response_msg.trajectory.joint_trajectory
        return joint_trajectory
    else:
        rospy.logwarn("Invalid MotionPlanResponse message format. Unable to extract joint trajectory.")
        return None

# Assuming you have a callback function for your MoveIt response
def moveit_response_callback(response_msg):
    joint_trajectory = extract_joint_trajectory(response_msg)
    if joint_trajectory:
        # Now you have the joint trajectory
        rospy.loginfo("Received joint trajectory: {}".format(joint_trajectory))
        # Your custom logic for handling the joint trajectory

# Assuming you have a subscriber to the relevant MoveIt topic
rospy.Subscriber('/move_group/planner/plan', MotionPlanResponse, moveit_response_callback)

# Ensure your node is spin()ning to keep the callbacks alive
rospy.spin()