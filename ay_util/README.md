ay_util
==================
ROS utility, including state validity checker (collision checker) for ROS, some launch files, models, etc.

`ay_util` is implemented with C++.


Author
==================
Akihiko Yamaguchi, http://akihikoy.net/


Requirements
==================
See `package.xml`.


Directories
==================

src_ros
----------------------------
ROS-dependent source files (C++).  Mainly, programs of ROS nodes are contained.


Build
==================
The repository directory should be in Catkin workspace (e.g. ~/catkin_ws/).
Build `ay_util` with `catkin_make`.

```
$ catkin_make
```

After `catkin_make`, you will find some executables in `devel/` directory.


Usage
==================

bx_gui
---------------------------
GUI launcher of Baxter system.

```
rosrun ay_util bx_gui.py
```

rqnb_gui
---------------------------
GUI launcher of Robotiq gripper system (no gripper).

```
rosrun ay_util rqnb_gui.py
```

state_validity_checker
---------------------------
State validity checker (collision checker).

Refer to launch files for Baxter and PR2:
- launch/baxter/bx_state_validity_checker.launch
- launch/pr2/pr2_state_validity_checker.launch

For examples of trajectory planning with state validity checker, see ay_trick.

Note that typically you do not have to launch `state_validity_checker` directly.  It is automatically launched from Baxter and PR2 systems.


Troubles
==================
Send e-mails to the author.
