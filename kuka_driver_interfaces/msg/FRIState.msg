# Message describing the state of the robot and FRI

# FRI session state enum
# IDLE = 0, MONITORING_WAIT = 1, MONITORING_READY = 2,COMMANDING_WAIT = 3, COMMANDING_ACTIVE = 4
int32 session_state

# FRI connection quality
# POOR = 0, FAIR = 1, GOOD = 2, EXCELLENT = 3
int32 connection_quality

# Safety state
# NORMAL_OPERATION = 0, SAFETY_STOP_LEVEL_0 = 1, SAFETY_STOP_LEVEL_1 = 2, SAFETY_STOP_LEVEL_2 = 3
int32 safety_state

# Client command mode
# NO_COMMAND_MODE = 0, POSITION = 1, WRENCH = 2, TORQUE = 3
int32 command_mode

# Control mode of the robot
# POSITION_CONTROL_MODE = 0, CART_IMP_CONTROL_MODE = 1, JOINT_IMP_CONTROL_MODE = 2, NO_CONTROL = 3
int32 control_mode

# Operation mode
# TEST_MODE_1 = 0, TEST_MODE_2 = 1, AUTOMATIC_MODE = 2
int32 operation_mode

# Drive state
# OFF = 0, TRANSITIONING = 1, ACTIVE = 2
int32 drive_state

# Overlay overlay_type
# NO_OVERLAY = 0, JOINT = 1, CARTESIAN = 2
int32 overlay_type

# Tracking performance of the robot
# Indicates how well the commanded robot is able to follow the commands of the FRI client.
# The best possible value is 1.0, when the robot executes the given commands instantaneously.
# The tracking performance is always 0 when the session state is not COMMANDING_ACTIVE.
float64 tracking_performance
