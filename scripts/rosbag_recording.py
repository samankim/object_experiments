#!/usr/bin/env python

# File: rosbag_recording.py
# ------------------------------------  
# Starts a service node that waits for a service call with
# the desired choreography. Then begins a  rosbag recording 
# of all nodes, triggers the robot choreography through actionlib
# , and stops the recording after a time buffer.
# Based on code from the ROS wiki tutorials "Writing a Simple 
# Action Client (Python)" and "Writing a Simple Service and
# Client (Python)."
#
# 7/11/18
# By: Samantha Kim

from __future__ import print_function
import rospy
# Brings in the SimpleActionClient
import actionlib
from object_experiments.srv import *
# Brings in the messages used by object experiments, including
# the ChoreographyAction goal and result messages
import object_experiments.msg
# For rosbag recording
import subprocess
import signal
import os

def choreography_client(choreo_name):
    # Creates the SimpleActionClient, passing the type of the action
    # (Choreography) to the constructor.
    client = actionlib.SimpleActionClient('choreography', object_experiments.msg.ChoreographyAction)
    # Waits until the action server has started up and started
    # listening for goals.
    print("Waiting for server...")
    client.wait_for_server()

    # Start the rosbag recording
    # Adapted from https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
    rospy.loginfo("Beginning rosbag recording.")
    command = "rosbag record -o '/media/ratchet/TOSHIBA EXT/kinect_bagfiles/knock_8_blocks_.1_kinect_2_hd' /kinect_2/hd/image_color_rect"
    rosbag_proc = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True)

    # Creates a goal to send to the action server.
    goal = object_experiments.msg.ChoreographyGoal()
    goal.choreography.data = choreo_name
    # Sends the goal to the action server.
    client.send_goal(goal)
    print("Goal sent.")
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Wait 3 seconds to ensure that all movement is captured
    d = rospy.Duration(3, 0)
    rospy.sleep(d)

    # Stop the rosbag recording
    # Adapted from https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
    terminate_ros_node("/record")

def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s + "_") or str == s):
            os.system("rosnode kill " + str)


def handle_experiment(ChoreographySrv):
    try:
        choreo_name = ChoreographySrv.choreography
        choreography_client(choreo_name)
        print ("Experiment has been executed.")
        return ChoreographySrvResponse(True)
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)
        return ChoreographySrvResponse(False)

def experiment_server():
    rospy.init_node('choreography_client')
    s = rospy.Service('start_experiment', ChoreographySrv, handle_experiment)
    print("Ready to begin experiments.")
    rospy.spin()

if __name__ == '__main__':
    experiment_server()
