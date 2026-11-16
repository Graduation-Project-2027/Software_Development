# State of the lift, CAN frame LIFT_STATUS (0x022) from the body controller (STM32F411).
# Two motors with encoders, homed on limit switches. Command it on /lift_joint/cmd_pos
# (std_msgs/Float64, metres above the home position) and home it with the lift/home service.

uint8 STATE_UNHOMED = 0        # after power-up: the first command homes it
uint8 STATE_HOMING = 1
uint8 STATE_IDLE = 2           # holding
uint8 STATE_MOVING = 3
uint8 STATE_FAULT = 4          # stopped; lift/home clears it

uint8 FAULT_NONE = 0
uint8 FAULT_HOMING_TIMEOUT = 1 # a limit switch was never reached
uint8 FAULT_STALL = 2          # a motor did not turn (jam, or encoder not counting)
uint8 FAULT_SKEW = 3           # left and right sides too far apart

std_msgs/Header header
bool online                    # a LIFT_STATUS frame arrived within status_timeout
uint8 state                    # STATE_*
uint8 fault                    # FAULT_*
bool homed
bool at_target                 # holding within tolerance of the commanded height
bool limit_a                   # limit switch A actuated
bool limit_b                   # limit switch B actuated
float32 position               # [m] above home, mean of both sides
float32 skew                   # [m] left minus right
uint8 heartbeat                # increments by one per frame
