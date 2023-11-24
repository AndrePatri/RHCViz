# RHCViz package

Minimal tool to visualize Receding Horizon Control solutions on RViz in realtime.

Dependencies:
- [rospkg](https://github.com/ros-infrastructure/rospkg)
- [RViz](https://github.com/ros-visualization/rviz)
- [pyyaml](https://pypi.org/project/PyYAML/)
- The tests run on two example robot description packages:
    - [Aliengo1](https://github.com/AndrePatri/unitree_ros). This is a fork of *unitree_ros* (note: install in your workspace only the ```aliengo_description``` subpackage).
    - [Centauro](https://github.com/ADVRHumanoids/iit-centauro-ros-pkg/tree/big_wheels_v2.10_optional_find)

ToDo:
- [ ] automatic number of robot models spawning with right tf and namespace
- [ ] add robot for current state
- [ ] namespace and tf should be {robot_name}/node_{i}