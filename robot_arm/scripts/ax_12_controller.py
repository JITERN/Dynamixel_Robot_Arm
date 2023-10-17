#!/usr/bin/env python3
from robot_arm.Ax12 import Ax12
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray

# e.g 'COM3' windows or '/dev/ttyUSB0' for Linux
Ax12.DEVICENAME = '/dev/ttyUSB0'

Ax12.BAUDRATE = 1_000_000

# sets baudrate and opens com port
Ax12.connect()

motor_ids = [1,2,3,4,5,6]
motors = [Ax12(motor_id) for motor_id in motor_ids]
goal_pos = []
got_loads = False
got_gripper_load = False
loads = []

rospy.init_node('ax12_controller')

def get_motor_load(data):
    global got_loads
    got_loads = True

def extract_data(data):
    global goal_pos
    if len(data.position) == len(motors):
        for motor, pos in zip(motors, data.position):
            goal_pos.append(int(((pos + 2.61) / (2 * 2.61)) * 1023))

def get_gripper_load(data):
    global got_gripper_load
    got_gripper_load = True


# Subscribe to the '/joint_states/goal' topic
rospy.Subscriber('/joint_states/goal', JointState, extract_data)
rospy.Subscriber('/command', Int16, get_motor_load)
rospy.Subscriber('/gripstatus', Int16, get_gripper_load)

joint_state_pub = rospy.Publisher('move_group/joint_states', JointState, queue_size=10)
load_pub = rospy.Publisher('/load', Int16MultiArray, queue_size=10)
gripper_stop_pos = rospy.Publisher('/joint_states/goal', JointState, queue_size=10)


def main(motors):
    """ sets goal position based on user input """
    global goal_pos
    global loads
    global got_loads
    global got_gripper_load
    for motor in motors:
        #motor.set_cw_compliance_slope(128)
        #motor.set_ccw_compliance_slope(128)
        motor.set_return_delay_time(0)
        print(float(motor.get_return_delay_time()))
        motor.set_moving_speed(150)
    motors[-1].set_moving_speed(50)  

    rate = rospy.Rate(10)  # Set the publishing rate (10 Hz in this case)
    while not rospy.is_shutdown():
        if goal_pos:
            for motor, pos in zip(motors, goal_pos):
                motor.set_goal_position(pos)
            goal_pos = []
        else:
            positions = [(((i.get_present_position()/ 1023) * (2 * 2.61)) - 2.61) for i in motors]
            #print("\nCurrent Positions: %s" % ', '.join(map(str, positions)))
            max_load = 150
            current_load = motors[-1].get_load()
            if current_load > max_load and current_load < 1024 and got_gripper_load is True:
                joint_state_msg.position[-1] = positions[-1]
                gripper_stop_pos.publish(joint_state_msg)
                block_size = positions[-1]
                if block_size > -1 and block_size < 1:
                    print("Big Dick")
                elif block_size <= -1 and block_size > -1.5:
                    print("Small Dick")
                else:
                    print("NO Dick")
                got_gripper_load = False
            
            # Publish motor positions to /joint_states topic
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.header.frame_id = "world"
            # joint_state_msg.name = ["motor_{}".format(i) for i in motor_ids]
            joint_state_msg.name = ["joint{}".format(i-1) for i in motor_ids]
            joint_state_msg.position = positions
            joint_state_pub.publish(joint_state_msg)
        
        # FOR DEBUG USE
        # if got_loads:
        #     for motor in motors:
        #         loads.append(motor.get_load())
        #     print(loads)
        #     load_arr = Int16MultiArray()
        #     load_arr.data = loads
        #     load_pub.publish(load_arr)
        #     got_loads = False
        #     loads = []
            
        rate.sleep()

if __name__ == '__main__':
    try:
        main(motors)
    except rospy.ROSInterruptException:
        pass


# disconnect
# my_dxl.set_torque_enable(0)
Ax12.disconnect()
