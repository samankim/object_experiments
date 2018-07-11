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

import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonInterface(object):
  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()

    ## Initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface',
                    anonymous=True)

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
    group.set_max_acceleration_scaling_factor(0.01)
    group.set_max_velocity_scaling_factor(0.01)


    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
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


  def execute_collision(self):   
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
    pre_coll_pose.position.z = -1.07752385597
    
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

    plan_and_execute_pose_goal(pre_coll_pose, true)
    plan_and_execute_pose_goal(post_coll_pose, true)
    plan_and_execute_pose_goal(init_pose, true)
    
    
  def plan_pose_goal(pose_goal, execute = true):
    # Add pose goals and execute path
    self.group.set_pose_target(pose_goal)
    plan = self.group.go(wait=True)
    
    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.group.clear_pose_targets()


def main():
  try:
    # Initialize MoveIt commander
    robot_commander = MoveGroupPythonInterface()
    # Execute collision
    robot_commander.execute_collision()
    print "============ Collision complete!"
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
