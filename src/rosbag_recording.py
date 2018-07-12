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

#!/usr/bin/env python
from __future__ import print_function
import rospy
# Brings in the SimpleActionClient
import actionlib
# Brings in the messages used by object experiments, including
# the Choreography goal and result messages
import object_experiments.msg
# For rosbag recording
import subprocess
import signal

def choreography_client(choreography):
    # Creates the SimpleActionClient, passing the type of the action
    # (Choreography) to the constructor.
    client = actionlib.SimpleActionClient('choreography', object_experiments.msg.ChoreographyAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Start the rosbag recording
    # Adapted from https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
    rospy.loginfo("Beginning rosbag recording.")
    command = "rosbag record -a"
    rosbag_proc = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=dir_save_bagfile)

    # Creates a goal to send to the action server.
    goal = object_experiments.msg.ChoreographyAction(choreography)
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Wait 3 seconds to ensure that all movement is captured
    d = rospy.Duration(3, 0)
    rospy.sleep(d)

    # Stop the rosbag recording
    # Adapted from https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
    rospy.loginfo("Stopping rosbag recording")
    rosbag_proc.send_signal(subprocess.signal.SIGINT)

def handle_experiment(choreography):
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('choreography_client')
        choreography_client(choreography)
        print "Experiment has been executed."
        return ChoreographySrvResponse(True)
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)
        return ChoreographySrvResponse(False)

def experiment_server():
    rospy.init_node('experiment_server')
    s = rospy.Service('experiment', ChoreographySrv, handle_experiment)
    print "Ready to begin experiments."
    rospy.spin()

if __name__ == '__main__':
    experiment_server()
