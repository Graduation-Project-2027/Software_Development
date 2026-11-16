"""AMR CAN protocol (hoverboard VARIANT_CAN + e-stop controller) and diff-drive kinematics.

Pure Python (no ROS imports) so it can be unit-tested off the robot.
The frame layout mirrors Hoverboard_Firmware/docs/hoverboard_can.dbc:
standard 11-bit IDs, little-endian payloads, n = CAN_NODE_ID of the board.
Every wheel value uses the firmware sign convention: positive = robot forward.
"""

import math
import struct
from dataclasses import dataclass

ESTOP_ID = 0x010
SAFETY_STATUS_ID = 0x011             # sent by the STM32F411 e-stop controller
BODY_CMD_ID = 0x020                  # stack light + buzzer request for the STM32F411
LIFT_CMD_ID = 0x021                  # lift command for the STM32F411
LIFT_STATUS_ID = 0x022               # lift state from the STM32F411
CMD_BASE = 0x200
ODOM_BASE = 0x180
MOTION_BASE = 0x280
STATUS_BASE = 0x380

_CMD = struct.Struct("<hh")          # rpm_left, rpm_right
_ODOM = struct.Struct("<ii")         # ticks_left, ticks_right
_MOTION = struct.Struct("<hhhh")     # rpm_left, rpm_right, current_left*100, current_right*100
_STATUS = struct.Struct("<HBbBBBB")  # batV*100, bat%, temp, err, flags, can_err, heartbeat
_SAFETY = struct.Struct("<BB")       # flags (bits 0-2) + cause (bits 4-6), heartbeat
_LIFT_CMD = struct.Struct("<BH")     # command, target [0.1 mm]
_LIFT_STATUS = struct.Struct("<BBhhBB")  # state, flags, position [0.1 mm], skew [0.1 mm], fault, heartbeat

LIFT_STOP = 0
LIFT_GOTO = 1
LIFT_HOME = 2

LIFT_UNHOMED = 0
LIFT_HOMING = 1
LIFT_IDLE = 2
LIFT_MOVING = 3
LIFT_FAULT = 4

CAUSE_NONE = 0
CAUSE_HOST_REQUEST = 1
CAUSE_BUTTON = 2
CAUSE_HOST_TIMEOUT = 3

# BODY_CMD byte 0 (stack light) and byte 1 (buzzer)
LIGHT_IDLE = 0                       # green
LIGHT_MOVING = 1                     # green + blinking amber
LIGHT_PAUSED = 2                     # amber
LIGHT_FAULT = 3                      # red
BUZZER_OFF = 0
BUZZER_SLOW = 1                      # one short beep per second (travel alarm)
BUZZER_FAST = 2                      # 5 Hz beeping (lifting, warning)

INT16_MIN = -32768
INT16_MAX = 32767


def cmd_id(node_id):
    return CMD_BASE + node_id


def odom_id(node_id):
    return ODOM_BASE + node_id


def motion_id(node_id):
    return MOTION_BASE + node_id


def status_id(node_id):
    return STATUS_BASE + node_id


@dataclass
class Status:
    battery_voltage: float
    battery_percent: int
    board_temperature: int
    error_left: int
    error_right: int
    motors_enabled: bool
    cmd_timeout: bool
    safe_state: bool
    estop_active: bool
    aux_input: bool
    battery_low: bool
    temp_warning: bool
    can_error_counter: int
    heartbeat: int


@dataclass
class SafetyStatus:
    estop_active: bool
    hoverboard_power: bool
    button_pressed: bool
    cause: int
    heartbeat: int


@dataclass
class LiftStatus:
    state: int
    homed: bool
    at_target: bool
    limit_a: bool
    limit_b: bool
    position: float      # [m] above home
    skew: float          # [m] left minus right
    fault: int
    heartbeat: int


def encode_lift_cmd(command, height_m=0.0):
    """LIFT_CMD payload. height_m is the target above the home switches (LIFT_GOTO only)."""
    tenth_mm = max(0, min(0xFFFF, int(round(height_m * 10000.0))))
    return _LIFT_CMD.pack(command, tenth_mm)


def decode_lift_status(data):
    state, flags, position, skew, fault, heartbeat = _LIFT_STATUS.unpack_from(data)
    return LiftStatus(
        state=state,
        homed=bool(flags & 0x01),
        at_target=bool(flags & 0x02),
        limit_a=bool(flags & 0x04),
        limit_b=bool(flags & 0x08),
        position=position / 10000.0,
        skew=skew / 10000.0,
        fault=fault,
        heartbeat=heartbeat,
    )


def encode_cmd(rpm_left, rpm_right):
    """HB_CMD payload. Values are rpm in the firmware's SPD_MODE."""
    clamp = lambda v: max(INT16_MIN, min(INT16_MAX, int(round(v))))
    return _CMD.pack(clamp(rpm_left), clamp(rpm_right))


def encode_estop(active):
    """ESTOP payload: 1 = stop (latched on every node), 0 = release."""
    return bytes([1 if active else 0])


def encode_body_cmd(light_mode, buzzer=BUZZER_OFF):
    """BODY_CMD payload: what the stack light and the buzzer show while not e-stopped."""
    if not LIGHT_IDLE <= light_mode <= LIGHT_FAULT:
        light_mode = LIGHT_IDLE
    if not BUZZER_OFF <= buzzer <= BUZZER_FAST:
        buzzer = BUZZER_OFF
    return bytes([light_mode, buzzer])


def decode_odom(data):
    """HB_ODOM -> (ticks_left, ticks_right)."""
    return _ODOM.unpack_from(data)


def decode_motion(data):
    """HB_MOTION -> (rpm_left, rpm_right, current_left [A], current_right [A])."""
    rpm_l, rpm_r, cur_l, cur_r = _MOTION.unpack_from(data)
    return rpm_l, rpm_r, cur_l / 100.0, cur_r / 100.0


def decode_status(data):
    bat_v, bat_pct, temp, err, flags, can_err, heartbeat = _STATUS.unpack_from(data)
    return Status(
        battery_voltage=bat_v / 100.0,
        battery_percent=bat_pct,
        board_temperature=temp,
        error_left=err & 0x0F,
        error_right=(err >> 4) & 0x0F,
        motors_enabled=bool(flags & 0x01),
        cmd_timeout=bool(flags & 0x02),
        safe_state=bool(flags & 0x04),
        estop_active=bool(flags & 0x08),
        aux_input=bool(flags & 0x10),
        battery_low=bool(flags & 0x20),
        temp_warning=bool(flags & 0x40),
        can_error_counter=can_err,
        heartbeat=heartbeat,
    )


def decode_safety_status(data):
    """SAFETY_STATUS of the e-stop controller (relay that powers the hoverboard board)."""
    flags, heartbeat = _SAFETY.unpack_from(data)
    return SafetyStatus(
        estop_active=bool(flags & 0x01),
        hoverboard_power=bool(flags & 0x02),
        button_pressed=bool(flags & 0x04),
        cause=(flags >> 4) & 0x07,
        heartbeat=heartbeat,
    )


def tick_delta(new, old):
    """Difference of two int32 hall counters, robust to wrap-around."""
    d = (new - old) & 0xFFFFFFFF
    return d - (1 << 32) if d >= (1 << 31) else d


def rpm_to_rad_s(rpm):
    return rpm * 2.0 * math.pi / 60.0


def rad_s_to_rpm(w):
    return w * 60.0 / (2.0 * math.pi)


class DiffDriveKinematics:
    """Two driven wheels on one axle through the robot centre (casters elsewhere).

    Wheel speeds are angular [rad/s], positive when the wheel rolls forward.
    """

    def __init__(self, wheel_radius, wheel_separation):
        self.r = wheel_radius
        self.half_track = wheel_separation / 2.0

    def inverse(self, v, wz):
        """Body twist -> (left, right) wheel angular speeds."""
        return (
            (v - wz * self.half_track) / self.r,
            (v + wz * self.half_track) / self.r,
        )

    def forward(self, left, right):
        """Wheel speeds (or angle increments) -> (v, wz) (or (distance, heading change))."""
        return (
            self.r * (left + right) / 2.0,
            self.r * (right - left) / (2.0 * self.half_track),
        )


class WheelOdometry:
    """Dead reckoning from the hall-tick counters of the two drive wheels.

    The counters restart at 0 whenever the board is powered again (e-stop relay,
    brown-out). A step that disagrees with the measured wheel speed is dropped and
    the next sample becomes the new baseline, so a restart never moves the pose.
    """

    def __init__(self, kinematics, ticks_per_rev):
        self.kin = kinematics
        self.ticks_per_rev = ticks_per_rev
        self.x = self.y = self.yaw = 0.0
        self.wheel_angle = [0.0, 0.0]
        self._last_ticks = None
        self._last_time = None

    def reset_baseline(self):
        """Forget the last counters (board offline): the next sample only sets the baseline."""
        self._last_ticks = None

    def update(self, ticks, rpm, now):
        """Integrate one (ticks_left, ticks_right) sample taken at time `now` [s].

        `rpm` is the measured (left, right) wheel speed, used to recognise counter
        restarts. Returns False when the sample was dropped as a counter jump.
        """
        if self._last_ticks is None:
            self._last_ticks, self._last_time = ticks, now
            return True
        dt = now - self._last_time
        deltas = [tick_delta(new, old) for new, old in zip(ticks, self._last_ticks)]
        self._last_ticks, self._last_time = ticks, now
        for delta, wheel_rpm in zip(deltas, rpm):
            expected = wheel_rpm * self.ticks_per_rev / 60.0 * dt
            if abs(delta - expected) > 0.5 * self.ticks_per_rev + abs(expected):
                return False

        angles = [d * 2.0 * math.pi / self.ticks_per_rev for d in deltas]
        distance, dyaw = self.kin.forward(*angles)
        heading = self.yaw + dyaw / 2.0
        self.x += distance * math.cos(heading)
        self.y += distance * math.sin(heading)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))
        for i, a in enumerate(angles):
            self.wheel_angle[i] += a
        return True


def choose_indication(requested_light, requested_buzzer, moving, fault, travel_buzzer=False):
    """Stack light and buzzer for BODY_CMD.

    A drive fault shows red; wheels being commanded shows "moving" whatever the
    mission layer asked for (teleop, docking moves); otherwise the request stands.
    """
    light, buzzer = requested_light, requested_buzzer
    if fault:
        light = LIGHT_FAULT
    elif moving and light in (LIGHT_IDLE, LIGHT_PAUSED):
        light = LIGHT_MOVING
    if travel_buzzer and light == LIGHT_MOVING and buzzer == BUZZER_OFF:
        buzzer = BUZZER_SLOW
    return light, buzzer
