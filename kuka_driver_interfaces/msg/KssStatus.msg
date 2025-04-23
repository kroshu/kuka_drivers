# Common constant
uint8 UNSPECIFIED = 0

# Control mode constants
uint8 JOINT_POSITION_CONTROL = 1
uint8 CARTESIAN_POSITION_CONTROL = 5

# Cycle time constants
uint8 RSI_4MS = 1
uint8 RSI_12MS = 2

# Operation mode constants
uint8 T1 = 1
uint8 T2 = 2
uint8 AUT = 3
uint8 EXT = 4

# Message fields
uint8 control_mode
uint8 cycle_time
bool drives_enabled
bool drives_powered
bool emergency_stop
bool guard_stop
bool in_motion
bool motion_possible
uint8 operation_mode
