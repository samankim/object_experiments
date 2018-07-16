# Current Issues
- rosbag recordings are very laggy
  - possible solutions:
    - start a new subprocess for each kinect and record a rosbag separately for each one
      - Functions to fix are in scripts/rosbag_recording.py, under choreography_client
- robotiq package does not compile
  - prevents gripper from being visualized
  - a gazebo problem?
- rad_ur10_stack does not compile
#### Future functionality
  - create an arg for the bagfile destination so that it does not need to be hardcoded
  - implement functionality for preempting with respect to the ActionServer (in move_group_python_interface.py)
  - incorporate gripper in move_group_python_interface.py
