# Basic gripper status flags.
# Note that the detailed states will be published as joint_states.

std_msgs/Header header

# true if there is no error, torque enabled, and the gripper can move
bool is_normal

# true if there is any error and the gripper cannot operate
bool is_error

# true if motor torque is ON and the gripper can move
bool torque_enabled
