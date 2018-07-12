# File: rosbag_recording.py
# ------------------------------------  
# Starts a rosbag recording of all nodes, triggers the robot
# choreography, and stops the recording after a time buffer.
# Based on code from the ROS wiki tutorial "Writing a Simple 
# Action Client (Python)".
# TODO: turn this into a C++ node to make a wrapper for rosbag::Recorder
#
# 7/11/18
# By: Samantha Kim

#! /usr/bin/env python
from __future__ import print_function
import rospy
# Brings in the SimpleActionClient
import actionlib
# Brings in the messages used by object experiments, including
# the Choreography goal and result messages
import object_experiments.msg

def start_recording():
	duration = rospy.get_param("duration")
	command = "rosbag_record --duration=" + str(duration) " -a"
	rosbag_record = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=dir_save_bagfile)

def stop_recording():


def choreography_client():

	choreography = rospy.get_param("choreography")

	# Creates the SimpleActionClient, passing the type of the action
	# (Choreography) to the constructor.
    client = actionlib.SimpleActionClient('choreography', object_experiments.msg.Choreography)
	# Waits until the action server has started up and started
	# listening for goals.
    client.wait_for_server()

	# Start the rosbag recording
    start_recording()

	# Creates a goal to send to the action server.
    goal = object_experiments.msg.Choreography(choreography)
	# Sends the goal to the action server.
    client.send_goal(goal)
	# Waits for the server to finish performing the action.
    client.wait_for_result()

	# Stop the rosbag recording
    stop_recording()


if __name__ == '__main__':
	try:
		# Initializes a rospy node so that the SimpleActionClient can
		# publish and subscribe over ROS.
		rospy.init_node('choreography_client')
		result = choreography_client()
		print "Program has been executed."
	except rospy.ROSInterruptException:
		print("Program interrupted before completion", file=sys.stderr)
