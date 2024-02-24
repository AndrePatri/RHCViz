# RHCViz package

Minimal tool to visualize Receding Horizon Control solutions on RViz in realtime.

Dependencies:
- [ros2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [RViz2](https://github.com/ros2/rviz)
- [pyyaml](https://pypi.org/project/PyYAML/)
- [numpy](https://pypi.org/project/numpy/)
- [urdf_parser_py](https://pypi.org/project/urdf-parser-py/)
- [PerfSleep](https://github.com/AndrePatri/PerfSleep)

- The tests run on two example robot description packages:
    - [Aliengo1](https://github.com/AndrePatri/unitree_ros/tree/ros2)
    - [Centauro](https://github.com/ADVRHumanoids/iit-centauro-ros-pkg/tree/big_wheels_v2.10_optional_find_ros2)

To run a simple test:
- first make sure to have installed all the required packages and to have sourced your ROS installation.
- run `reset && python3 test_multirobot_visualization.py --robot_type aliengo` in a terminal. This will wait for something to publish handshake data and RHC states and robot states.
- in another terminal run `reset && python3 dummy_publisher_all.py --robot_type aliengo --n_rhc_nodes 5`. This will publish handshake data (basically signaling the main scripts how many robots it should signal), constant base pose and random joint positions.

Note: it is up to the user to implement a bridge which connects its Receding Horizon Controller with RHCViz. By default, RHCViz uses some naming conventions for its *bridge* topics, defined in `utils/namings.py`. Example implementations of dummy bridges are provided in the test folder.
