# Object Experiments
The object_experiments package contains scripts for running experiments involving the UR10. Launching the file `experiment.launch` will initialize communication with the UR10 and the Kinects, record a rosbag of all currently running topics, run pre-defined choreography on the UR10 that may interact with objects on the experiment surface, and automatically stop the rosbag recording.

### To run an experiment:

1. Turn on the UR10 and initialize it
2. Place necessary objects for experiment inside UR10 gripper prior to running the script (e.g. pick up pencil for "knock_blocks" experiment)
3. In separate terminal windows:
  ```
  $ roslaunch object_experiments experiment.launch
  $ rosservice call /start_experiment "choreography:'desired-choreography-name'"
  ```

### To add new choreography:

1. Go to the /scripts folder
2. Open 'move_group_python_interface.py' in your preferred text editor
3. Under the MoveGroupPythonInterface class, add another function with the name of your choreography. The 'knock_blocks' choreography is there as an example.
4. You may define a series of pose goals and joint states that the UR10 will execute, along with the desired velocity and acceleration of the movement.
5. In the execute_choreography function, add another if/elif statement detailing the name that will call your new choreography.
```python
elif goal == "name-of-choreography"
  robot_commander.choreography-function-name()
```

### To hard-code a new pose in 'move_group_python_interface.py':

1. Move the robot to the desired pose using the teaching pendant, calling commands using the MoveIt commander, or running a script using experiment.launch.
2. Call `$ rosservice call /start_experiment "choreography:'get_formatted_current_pose'"`. This will generate text with the description of the current pose.
3. The generated text can then be copied and pasted into 'move_group_python_interface.py' with the precise description of the robot's pose. Replace `your_pose_name` with the desired name of the recorded pose.

### To install
1. Download the package to ~/catkin_ws/src
2. Navigate back to ~/catkin_ws (`$ cd ..`)
3. Run `catkin_make`
