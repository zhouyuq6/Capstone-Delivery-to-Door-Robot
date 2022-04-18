boxer_desktop
===============

This repository contains packages for customizing rviz for use with the Boxer 2.4.

The `boxer_description` package is necessary to use this repository.

Example Usage
--------------

View a static model of the robot to verify the link positions:

```bash
roslaunch boxer_viz view_model.launch
```

Monitor a real Boxer:

```bash
export ROS_MASTER_URI=http://cpr-boxer01:11311
roslaunch boxer_viz view_robot.launch
```
