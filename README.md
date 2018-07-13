#Object Experiments

###To run an experiment:
In separate terminal windows:
  ```
  $ roslaunch object_experiments experiment.launch
  $ rosservice call /start_experiment "choreography:'*Desired-choreography-name*'"
  ```

###To add new choreography:

1. Go to the /scripts folder
2. Open 'move_group_python_interface.py' in your preferred text editor
3. Under the MoveGroupPythonInterface class, add another function with the name of your choreography. The choreography 'knock_blocks' is there as an example.
4. You may define a series of pose goals and joint states that the UR10 will execute along with the desired velocity and acceleration of the movement.
5. In the execute_choreography function, add another if/elif statement detailing the name that will call your new choreography.
```python
elif goal == "*name-of-choreography*"
  robot_commander.*choreography-function-name*()
```

###To create a new pose based on the current pose of the robot:

Move the robot to the desired pose.
The generated text can then be copied and pasted into the code with the precise description of the robot's pose.
