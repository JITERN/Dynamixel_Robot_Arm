#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL



def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "arm_no_grip"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        print("go to joint state")
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go_to_pose_goal(self, x, y, z, qx, qy, qz,qw):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        # print("go to pose goal")
        move_group = self.move_group

        # x=float(input("X: "))
        # y=float(input("Y: "))
        # z=float(input("Z: "))
        # w=float(input("W: "))


        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        # qx = np.sin(np.pi / 2.0)
        # qy = 0.0
        # qz = 0.0
        # qw = np.cos(np.pi / 2.0)

        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy
        pose_goal.orientation.z = qz


        pose_goal.orientation.w = qw

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        move_group.set_planning_time(10)
        move_group.set_pose_target(pose_goal)
        # print("x: ", pose_goal.position.x)
        # print("y: ", pose_goal.position.y)
        # print("z: ", pose_goal.position.z)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        print("plan cartesian path")
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        
        # Note: We are just planning, not asking move_group to actually move the robot yet:
        print(waypoints)
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        print("display trajectory")
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        print("execute plan")
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        print("wait for state update")
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_hand"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11  # above the panda_hand frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )
# Generate a valid end effector pose
    def show_current_ef_pose(self):

        group_name = "gripper"  # Replace with your planning group name
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # Get the final end effector pose
        final_end_effector_pose = move_group.get_current_pose().pose

        # Print the final end effector pose
        print("Final End Effector Pose:")
        print("Position (x, y, z):", final_end_effector_pose.position.x, final_end_effector_pose.position.y, final_end_effector_pose.position.z)
        print("Orientation (x, y, z, w):", final_end_effector_pose.orientation.x, final_end_effector_pose.orientation.y, final_end_effector_pose.orientation.z, final_end_effector_pose.orientation.w)

        return()


# # User input final end effector position, move_group.plan() will plan trajectory, 
# then print out final joints excluding the gripper joint.
    def set_pose(self):
        from moveit_commander import MoveGroupCommander

        # Initialize the MoveGroupCommander for your robot arm
        group = MoveGroupCommander("arm_no_grip")

        # Define the desired end-effector pose
        # desired_pose = geometry_msgs.msg.Pose()
        # desired_pose.position.x = -0.0599992065892  # Set the x-coordinate of the position
        # desired_pose.position.y = -0.0285018898606  # Set the y-coordinate of the position
        # desired_pose.position.z = 0.345499895303  # Set the z-coordinate of the position
        # desired_pose.orientation.w = 0  # Set the orientation as a quaternion
        home_pose = [-0.0599648337688,-0.0285018898606,0.345499895303]
        goal1_pose = [-0.0599908811538, -0.0284971194932, 0.345502939487]

        end_effector_link = "link5"
        # Set the target pose for the end effector
        group.set_position_target(home_pose, end_effector_link) 

        success = group.go(wait = True)
        print(success)
        # Plan and execute the motion
        

    def plan_robot_position(self, x, y, z, qx = 0, qy = 0, qz = 0, qw= 1):
        print("go to pose goal")
        move_group = self.move_group

        # x=float(input("X: "))
        # y=float(input("Y: "))
        # z=float(input("Z: "))
        # w=float(input("W: "))


        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy
        pose_goal.orientation.z = qz
        pose_goal.orientation.w = qw

        print("Goal position")
        print("x: ", pose_goal.position.x)
        print("y: ", pose_goal.position.y)
        print("z: ", pose_goal.position.z)
        move_group.set_pose_target(pose_goal)
    
        # Plan a motion from the current state to the target state
        plan = move_group.plan()
        # print(plan)

        # Check if a valid plan is found
        # if plan[0]:
        #     # Extract the joint positions from the trajectory
        #     final_joint_positions = plan.joint_trajectory.points[-1].positions
        #     print("Final Joint Positions:", final_joint_positions.points.positions)
        #     print ()
        #     # break
        # else:
        #     print("No valid plan found!")
        # # except KeyError:
        # #         print("Invalid input. Final position cannot be executed. Please re-enter positions below:")
        move_group.clear_pose_targets()

        return (plan)
    
    def gripper_open(self):
         group_name = "gripper"  # Replace with the name of your planning group
         move_group = moveit_commander.MoveGroupCommander(group_name)

         
         joint_goal = move_group.get_current_joint_values()
         joint_goal[0] = 0.0
       

         move_group.go(joint_goal, wait=True)

         move_group.stop()
 
         # For testing:
         current_joints = move_group.get_current_joint_values()
        #  print("This is the current joint for gripper: ")
        #  print(current_joints)
         return all_close(joint_goal, current_joints, 0.01)
    
    def modified_go_to_pose_goal(self, x, y, z, qx, qy, qz,qw):

        from moveit_commander import MoveGroupCommander

        # Initialize the MoveGroupCommander for your robot arm
        move_group = MoveGroupCommander("arm_no_grip")
        pose_goal = geometry_msgs.msg.Pose()

        count = 0
        angle = 90
        while angle <= 271:

            qx = 0.0
            qy = 0.0
            qz = np.sin(angle / 2.0)
            qw = np.cos(angle / 2.0)

            pose_goal.orientation.x = qx
            pose_goal.orientation.y = qy
            pose_goal.orientation.z = qz
            pose_goal.orientation.w = qw

            pose_goal.position.x = x
            pose_goal.position.y = y
            pose_goal.position.z = z

            move_group.set_planning_time(1)
            move_group.set_pose_target(pose_goal)
            success = move_group.go(wait=True)
            angle += 1
            count += 1

            if success == True:
                break
                
            
        print(count)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()

        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


    

    def gripper_close(self):

         move_group = self.move_group
         # Create a `MoveGroupCommander` object for the robot arm's planning group
         group_name = "gripper"  # Replace with the name of your planning group
         move_group = moveit_commander.MoveGroupCommander(group_name)

         ## BEGIN_SUB_TUTORIAL plan_to_joint_state
         ##
         ## Planning to a Joint Goal
         ## ^^^^^^^^^^^^^^^^^^^^^^^^
         ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
         ## thing we want to do is move it to a slightly better configuration.
         ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
         # We get the joint values from the group and change some of the values:
         joint_goal = move_group.get_current_joint_values()
        #  hanstest = float(input("gripper:" ))
         joint_goal[0] = -0.926901367978
        #  joint_goal[1] = -tau / 8
        #  joint_goal[2] = 0
        #  joint_goal[3] = -tau / 4
        #  joint_goal[4] = 0
        #  joint_goal[5] = tau / 6  # 1/6 of a turn
        #  joint_goal[6] = 0  

         # The go command can be called with joint values, poses, or without any
         # parameters if you have already set the pose or joint target for the group
         move_group.go(joint_goal, wait=True)

         # Calling ``stop()`` ensures that there is no residual movement
         move_group.stop()
 
         ## END_SUB_TUTORIAL

         # For testing:
         current_joints = move_group.get_current_joint_values()
        #  print("This is the current joint for gripper: ")
        #  print(current_joints)
         return all_close(joint_goal, current_joints, 0.01)

    def go_arm(j0,j1,j2,j3,j4):
        
        group_name = "arm_no_grip"  # Replace with the name of your planning group
        move_group = moveit_commander.MoveGroupCommander(group_name)
    
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = j0
        joint_goal[1] = j1
        joint_goal[2] = j2
        joint_goal[3] = j3
        joint_goal[4] = j4
     
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL
        return all_close(joint_goal, 0.01)
    
    def go_grip(j5):
        
        group_name = "gripper"  # Replace with the name of your planning group
        move_group = moveit_commander.MoveGroupCommander(group_name)
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = j5
        
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL
        return all_close(joint_goal, 0.01)

def callback(data) :
    global information,size,weight
    size,weight = data.data.split(",")
    print("data is", size,weight)

def color_detected(data) :
    global color
    color = data.data


def main():
    try:
    #     print("----------------------------------------------------------")
    #     print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
    #     print("----------------------------------------------------------")
    #     print("Press Ctrl-D to exit at any time")
    #     print("")
    #     input(
    #         "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        # )
        global size,weight
        global color
        
        tutorial = MoveGroupPythonInterfaceTutorial()
        pub = rospy.Publisher("/command", Int16, queue_size=10)
        sub = rospy.Subscriber("/obj_status", String, callback)  
        ready_color_detection = rospy.Publisher("/ready_color_detection", Int16, queue_size=10)
        color_sub = rospy.Subscriber("/color", String, color_detected)  



        # Copy paste preset values of each positions, can see from "robot_arm_description_urdfv6.srdf" --> GROUP STATES

        # arm_on_grip ---->[j0,j1,j2,j3,j4]
        # gripper --->[j5]
        
        while True:
            move_group = moveit_commander.MoveGroupCommander("arm_no_grip")
            gripper_group = moveit_commander.MoveGroupCommander("gripper")
            move_group.set_named_target("home")
            move_group.go(wait=True)
            gripper_group.set_named_target("gripper_open")
            gripper_group.go(wait=True)

            input("Press 'Enter' to start execute if load is ready.")
        
            move_group.set_named_target("pickup_1")
            move_group.go(wait=True)

            ready_color_detection.publish(1)

            gripper_group.set_named_target("gripper_close")
            gripper_group.go(wait=True)

            move_group.set_named_target("measure_weight")
            move_group.go(wait=True)
            size = ""
            weight = ""
            color = ""
            pub.publish(0)

            while len(size) == 0:
                pass
            
            if color == "BLUE":
                print("Color is BLUE.")
                if weight == "heavy":
                    print("Weight is heavy.")
                    if size == "big":
                        print("Size is big")
                        move_group.set_named_target("goal_1")
                        move_group.go(wait=True)
                        print("reached goal 1")
                        gripper_group.set_named_target("gripper_open")
                        gripper_group.go(wait=True)
                    elif size == "small":
                        print("Size is small")
                        move_group.set_named_target("goal_4")
                        move_group.go(wait=True)
                        print("reached goal 4")
                        gripper_group.set_named_target("gripper_open")
                        gripper_group.go(wait=True)
                    else:
                        print("size is not big or small-->",size)

                elif weight == "light":
                    print("Weight is light.")
                    if size == "big":
                        print("Size is big")
                        move_group.set_named_target("goal_2")
                        move_group.go(wait=True)
                        print("reached goal 2")   
                        gripper_group.set_named_target("gripper_open")
                        gripper_group.go(wait=True)
                    elif size == "small":
                        print("Size is small")
                        move_group.set_named_target("goal_3")
                        move_group.go(wait=True)
                        print("reached goal 3")
                        gripper_group.set_named_target("gripper_open")
                        gripper_group.go(wait=True)

                    else:
                        print("size is not big or small-->",size)
                else:
                    print("weight is not heavy or light-->",weight)

            elif color == "LIGHT_GRAY":
                print("Color is LIGHT_GRAY.")
                if weight == "heavy":
                    print("Weight is heavy.")
                    if size == "big":
                        print("Size is big")
                        move_group.set_named_target("goal_6")
                        move_group.go(wait=True)
                        print("reached goal 6")
                        gripper_group.set_named_target("gripper_open")
                        gripper_group.go(wait=True)
                    elif size == "small":
                        print("Size is small")
                        move_group.set_named_target("goal_5")
                        move_group.go(wait=True)
                        print("reached goal 5")
                        gripper_group.set_named_target("gripper_open")
                        gripper_group.go(wait=True)
                    else:
                        print("size is not big or small-->",size)

                elif weight == "light":
                    print("Weight is light.")
                    if size == "big":
                        print("Size is big")
                        move_group.set_named_target("goal_8")
                        move_group.go(wait=True)
                        print("reached goal 8")   
                        gripper_group.set_named_target("gripper_open")
                        gripper_group.go(wait=True)
                    elif size == "small":
                        print("Size is small")
                        move_group.set_named_target("goal_7")
                        move_group.go(wait=True)
                        print("reached goal 8")
                        gripper_group.set_named_target("gripper_open")
                        gripper_group.go(wait=True)

                    else:
                        print("size is not big or small-->",size)
                else:
                    print("weight is not heavy or light-->",weight)
            user_input = input("Press 'Q' to exit, press 'Enter' to execute robot arm if next load is ready: ")

            if user_input.lower() == "q":
                print("good bye")
                break





    #     input(
    #         "============ Press `Enter` to execute a movement using a joint state goal ..."
    #     )
        # tutorial.go_to_joint_state()

    # #     input("============ Press `Enter` to execute a movement using a pose goal ...")
        # tutorial.go_to_pose_goal()
    #     print("goal arrived, preparing to grip")

    #     # TIGHTEN GRIP FUNCTION
    #     panda_hand_group = moveit_commander.MoveGroupCommander("panda_hand")
    #     grip_goal =[0.0,0.0]
    #     panda_hand_group.go(grip_goal, wait=True)
    #     print("gripper completed, please select destination")

    #     tutorial.go_to_pose_goal()
    #     print("reached destination, releasing grip")

    #     # LOOSEN GRIP FUNCTION
    #     grip_goal =[0.04,0.04]
    #     panda_hand_group.go(grip_goal, wait=True)
    #     print("gripper completed, please select destination")

        #generate a random valid end effector pose
        # tutorial.show_current_ef_pose()
        # tutorial.set_pose()
        # input("Press 'Enter' to move to goal1")
        # tutorial.go_to_pose_goal(*goal1)
        # print(" Succeed moving to goal1.")


        # # input("Press 'Enter' to close gripper")
        # tutorial.gripper_close()
        # print(" Succeed close gripper.")        

        # # input("Press 'Enter' to move to home")
        # tutorial.go_to_pose_goal(*goal2)
        # print(" Succeed moving to goal2.")        
        # # # tutorial.execute_plan(plan1)

        #modified go to pose goal
        # tutorial.modified_go_to_pose_goal(*goal1)
        # # input("Press 'Enter' to open gripper")
        # tutorial.gripper_open()
        # print(" Succeed open gripper.")  
        # # input("Press 'Enter' to Grip")
        # tutorial.go_gripper()



        # input("Press 'Enter' to release gripper")
        # tutorial.go_gripper()

        # input("============ Press `Enter` to plan and display a Cartesian path ...")
        # cartesian_plan, fraction = tutorial.plan_cartesian_path()


        # input(
        #     "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        # )
        # tutorial.display_trajectory(cartesian_plan)

        # input("============ Press `Enter` to execute a saved path ...")
        # tutorial.execute_plan(cartesian_plan)

    #     input("============ Press `Enter` to add a box to the planning scene ...")
        # tutorial.add_box()
 
    #     input("============ Press `Enter` to attach a Box to the Panda robot ...")
        # tutorial.attach_box()

    #     input(
    #         "============ Press `Enter` to plan and execute a path with an attached collision object ..."
    #     )
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        # tutorial.execute_plan(cartesian_plan)

    #     input("============ Press `Enter` to detach the box from the Panda robot ...")
        # tutorial.detach_box()

    #     input(
    #         "============ Press `Enter` to remove the box from the planning scene ..."
    #     )
        # tutorial.remove_box()

    #     print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/noetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL