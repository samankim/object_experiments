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

# 7/11/18: Updated for use by Robust Autonomy and Decisions Group by Samantha Kim
# Action code based on ROS wiki tutorial "Writing a Simple Action Server using the
# Execute Callback (Python)".

import sys
import copy
import rospy
import tf
import actionlib
import object_experiments.msg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MoveGroupPythonInterface(object):
    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

        ## Initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the Panda
        ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
        ## you should change this value to the name of your robot arm planning group.
        ## This interface can be used to plan and execute motions on the Panda:
        group_name = "manipulator"
        group = moveit_commander.MoveGroupCommander(group_name)
        
        # Initialize velocity and acceleration scaling factors to prevent
        # overly fast movements. Can be changed later using the go_to_pose_goal
        # and go_to_joint_state functions.
        group.set_max_acceleration_scaling_factor(0.1)
        group.set_max_velocity_scaling_factor(0.1)


        ## Create a `DisplayTrajectory`_ publisher which may be used to publish
        ## trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                                                                     moveit_msgs.msg.DisplayTrajectory,
                                                                                                     queue_size=20)

        # Print name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # Print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # List of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        # Print the state of the robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names




    def knock_blocks(self):
        """
        Function: knock_blocks
        ----------------------------------
        UR10 will go to a pre-collision pose, bend at the elbow joint,
        colliding with the block setup, and return to a neutral pose.
        Poses were found using the teaching pendant.
        """

        # Init pose
        init_pose = geometry_msgs.msg.Pose()
        init_pose.position.x = -0.488798937651
        init_pose.position.y = 0.104866858129
        init_pose.position.z = -1.0074033753

        init_pose.orientation.x = 0.495021656851
        init_pose.orientation.y = 0.516180354965
        init_pose.orientation.z = -0.48224425657
        init_pose.orientation.w = 0.505916868074


        # Pre-collision pose
        pre_coll_pose = geometry_msgs.msg.Pose()
        pre_coll_pose.position.x = -0.558980015093
        pre_coll_pose.position.y = 0.290542710322
        pre_coll_pose.position.z = -1.04752385597
        
        pre_coll_pose.orientation.x = 0.485564471115
        pre_coll_pose.orientation.y = 0.524631938133
        pre_coll_pose.orientation.z = -0.503994513944
        pre_coll_pose.orientation.w = 0.484745297859

        # Post-collision pose
        post_coll_pose = geometry_msgs.msg.Pose()
        post_coll_pose.position.x = -0.397289172218
        post_coll_pose.position.y = 0.290860833622
        post_coll_pose.position.z = -1.07770150547

        post_coll_pose.orientation.x = 0.485054814234
        post_coll_pose.orientation.y = 0.524070568573
        post_coll_pose.orientation.z = -0.504404664407
        post_coll_pose.orientation.w = 0.485448275813

        
        self.go_to_pose_goal(pre_coll_pose, .1, .1)

        # Post-collision joint state
        post_coll_joint_goal = self.group.get_current_joint_values()
        post_coll_joint_goal[3] -= pi / 6

        self.go_to_joint_state(post_coll_joint_goal, .1, .1)

        self.go_to_pose_goal(init_pose, .1, .1)


    def go_to_joint_state(self, joint_goal, velocity, acceleration):
        """
        Function: go_to_joint_state
        ------------------------------------    
        Moves the robot to the specified joint state with the
        specified velocity and acceleration. Velocity
        and acceleration are values between [0,1], corresponding
        to the scaling factor for the reduction of the maximum 
        joint velocity and acceleration.
        """

        # Set velocity and acceleration scaling factors. 
        self.group.set_max_velocity_scaling_factor(velocity)
        self.group.set_max_acceleration_scaling_factor(acceleration)
    
        self.group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()




    def go_to_pose_goal(self, pose_goal, velocity, acceleration):
        """
        Function: go_to_pose_goal
        ------------------------------------    
        Plans a pose goal and executes the path. This method is preferable 
        to cartesian path planning and execution because velocity and 
        acceleration limitations can be set.

        """
        # Set velocity and acceleration scaling factors. 
        self.group.set_max_velocity_scaling_factor(velocity)
        self.group.set_max_acceleration_scaling_factor(acceleration)
        
        # Add pose goals and execute path
        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=True)
        
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()


    

    def get_formatted_current_pose(self, pose_name):
        """
        Function: get_formatted_current_pose
        ------------------------------------  
        Prints to screen the current pose of the robot in a format that allows 
        for easy hardcoding of a particular pose.
        """


        current_pose = self.group.get_current_pose()
        print pose_name + " = geometry_msgs.msg.Pose()"
        print pose_name + ".position.x = " + str(current_pose.pose.position.x)
        print pose_name + ".position.y = " + str(current_pose.pose.position.y)
        print pose_name + ".position.z = " + str(current_pose.pose.position.z)
        print pose_name + ".orientation.x = " + str(current_pose.pose.orientation.x)
        print pose_name + ".orientation.y = " + str(current_pose.pose.orientation.y)
        print pose_name + ".orientation.z = " + str(current_pose.pose.orientation.z)
        print pose_name + ".orientation.w = " + str(current_pose.pose.orientation.w)
     
 

class Choreography(object):
    """
    Set up an Action Server that expects ChoreographyAction messages.
    When a ChoreographyAction goal is received, it executes the type of 
    choreography specified by the message.
    """


    feedback = object_experiments.msg.ChoreographyFeedback()
    result = object_experiments.msg.ChoreographyResult()
    success = True

    def __init__(self, name):
        self.action_name = name
        self.server = actionlib.SimpleActionServer(self.action_name, 
                                                    object_experiments.msg.ChoreographyAction, 
                                                    self.execute, 
                                                    auto_start = False)
        self.server.start()
        print("ActionServer initialized.")

    def execute(self,goal):
        self.success = True
        rospy.loginfo('Starting choreography: %s' % (goal))
        
        execute_choreography(goal)

        if self.success:
                rospy.loginfo('%s: Succeeded' % self.action_name)
                self.server.set_succeeded(self.result)

    def check_preempt(self):
        if self.server.is_preempt_requested():
                rospy.loginfo('%s: Preemepted' % self.action_name)
                self.server.set_preempted()
                self.success = False
                return False
        return True
        


def execute_choreography(goal):
    """
    Function: execute_choreography
    ------------------------------------  
    Executes the choreography specified as a goal.
    Additional choreography options should be included here
    as if/elif cases.
    """
    try:
        # Initialize MoveIt commander
        robot_commander = MoveGroupPythonInterface()
        # Execute choreography
        print("Executing: " + goal.choreography.data)
        if goal.choreography.data == "knock_blocks":
            robot_commander.knock_blocks()
        elif goal.choreography.data == "get_formatted_current_pose":
            robot_commander.get_formatted_current_pose("your_pose_name")
        print "============ Choreography complete!"
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return



if __name__ == '__main__':
    rospy.init_node('choreography')
    server = Choreography(rospy.get_name())
    rospy.spin()
