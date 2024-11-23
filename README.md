# knob_kuka_ws
required pkg: ros-noetic-rosserial

## Run demo in with fake topics:

```
$ roslaunch knob_robot_control demo.launch 
```

### ROS topics:
Currently the rostopics are still the one from kuka iiwa, modify when franka is ready.

#### position forward control:
**/knob_state ----> /fri_cartesian_command**

/fri_cartesian_command: robot pose command controlled via teleoparation interface, controlled axis can be changed with GUI or rosparam /controlled_axis \
/knob_state: read from current knob position \

/fri_cartesian_pose: current pose published from robot, only used when reset pose command, can be activated using GUI button "reset target pose"

#### force feedback:
**/tcp_force <---- /tcp_wrench**

/tcp_force: 1D force value to command the knoob\
/tcp_wrench: 6D force torque from current robot TCP

### ROS params:
/clamp_force_threshold_max \
/clamp_force_threshold_min \
/controlled_axis: GUI options for position control on 3 dimenstions x, y or z \
/fake_tcp_force: mannually set fake robot tcp force value \
/position_factor: can be changed using GUI \
/reset_pose: set to True for 3 seconds when pressing the GUI button \
/tcp_force_feedback_ratio: can be changed using GUI \
/tcp_force_offset



## Real knob:

```
$ roslaunch knob_robot_control rosserial.launch
```

## Real Robot Demo

Modify with real franka robot.


## GUI Demo

see script/qt_test.py

