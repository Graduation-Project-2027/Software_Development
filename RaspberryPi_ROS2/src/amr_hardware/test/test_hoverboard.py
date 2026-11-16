"""Protocol, kinematics and odometry tests (pure Python, no ROS or CAN needed).

    cd RaspberryPi_ROS2/src/amr_hardware && python3 -m pytest test
"""

import math
import os
import struct
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from amr_hardware import hoverboard as hb  # noqa: E402


def test_frame_ids_follow_the_node_id():
    assert hb.cmd_id(1) == 0x201
    assert hb.odom_id(1) == 0x181
    assert hb.motion_id(1) == 0x281
    assert hb.status_id(1) == 0x381
    assert hb.status_id(5) == 0x385


def test_cmd_is_little_endian_rounded_and_clamped():
    assert hb.encode_cmd(100, -100) == struct.pack("<hh", 100, -100)
    assert hb.encode_cmd(1.6, -1.6) == struct.pack("<hh", 2, -2)
    assert hb.encode_cmd(1e9, -1e9) == struct.pack("<hh", 32767, -32768)


def test_estop_payload():
    assert hb.encode_estop(True) == b"\x01"
    assert hb.encode_estop(False) == b"\x00"


def test_body_cmd_payload_rejects_unknown_values():
    assert hb.encode_body_cmd(hb.LIGHT_MOVING, hb.BUZZER_SLOW) == b"\x01\x01"
    assert hb.encode_body_cmd(hb.LIGHT_FAULT) == b"\x03\x00"
    assert hb.encode_body_cmd(99, 99) == b"\x00\x00"


def test_decode_motion_and_odom():
    assert hb.decode_odom(struct.pack("<ii", -5, 123456)) == (-5, 123456)
    rpm_l, rpm_r, cur_l, cur_r = hb.decode_motion(struct.pack("<hhhh", 60, -60, 250, -125))
    assert (rpm_l, rpm_r) == (60, -60)
    assert cur_l == pytest.approx(2.5)
    assert cur_r == pytest.approx(-1.25)


def test_decode_status_fields_and_flags():
    data = struct.pack("<HBbBBBB", 3912, 76, -3, 0x41, 0b0110_1001, 7, 200)
    s = hb.decode_status(data)
    assert s.battery_voltage == pytest.approx(39.12)
    assert s.battery_percent == 76
    assert s.board_temperature == -3
    assert (s.error_left, s.error_right) == (1, 4)
    assert s.motors_enabled and s.estop_active and s.battery_low and s.temp_warning
    assert not (s.cmd_timeout or s.safe_state or s.aux_input)
    assert (s.can_error_counter, s.heartbeat) == (7, 200)


def test_decode_safety_status_matches_the_firmware_layout():
    # bit0 active, bit1 power, bit2 button, bits 4-6 cause (protocol.h statusByte0)
    s = hb.decode_safety_status(bytes([0x01 | 0x04 | (hb.CAUSE_BUTTON << 4), 42]))
    assert s.estop_active and s.button_pressed and not s.hoverboard_power
    assert s.cause == hb.CAUSE_BUTTON and s.heartbeat == 42
    s = hb.decode_safety_status(bytes([0x02, 0]))
    assert s.hoverboard_power and not s.estop_active and s.cause == hb.CAUSE_NONE


def test_short_frames_raise_struct_error():
    with pytest.raises(struct.error):
        hb.decode_status(b"\x00\x01")
    with pytest.raises(struct.error):
        hb.decode_safety_status(b"\x00")


def test_tick_delta_wraps_like_int32():
    assert hb.tick_delta(10, 4) == 6
    assert hb.tick_delta(-2147483648, 2147483647) == 1
    assert hb.tick_delta(2147483647, -2147483648) == -1


def test_kinematics_round_trip():
    kin = hb.DiffDriveKinematics(0.08, 0.5)
    left, right = kin.inverse(0.4, 0.6)
    v, wz = kin.forward(left, right)
    assert v == pytest.approx(0.4)
    assert wz == pytest.approx(0.6)
    # Turning left on the spot: left wheel backwards, right wheel forwards.
    left, right = kin.inverse(0.0, 1.0)
    assert left < 0 < right


def _drive(odo, ticks_per_step, steps, start=(0, 0), dt=0.02):
    left, right = start
    rpm = [t / odo.ticks_per_rev * 60.0 / dt for t in ticks_per_step]
    for i in range(steps + 1):
        assert odo.update((left, right), rpm, i * dt)
        left += ticks_per_step[0]
        right += ticks_per_step[1]


def test_odometry_straight_line():
    odo = hb.WheelOdometry(hb.DiffDriveKinematics(0.08, 0.5), 90.0)
    _drive(odo, (3, 3), 30)                       # one wheel revolution
    assert odo.x == pytest.approx(2 * math.pi * 0.08)
    assert odo.y == pytest.approx(0.0, abs=1e-9)
    assert odo.yaw == pytest.approx(0.0, abs=1e-9)
    assert odo.wheel_angle[0] == pytest.approx(2 * math.pi)


def test_odometry_turn_on_the_spot_is_counter_clockwise_for_right_forward():
    odo = hb.WheelOdometry(hb.DiffDriveKinematics(0.08, 0.5), 90.0)
    _drive(odo, (-1, 1), 45)
    assert odo.x == pytest.approx(0.0, abs=1e-9)
    assert odo.yaw == pytest.approx(math.pi * 0.08 / 0.25)   # wheel arc / half track
    assert odo.yaw > 0


def test_odometry_ignores_a_board_restart():
    odo = hb.WheelOdometry(hb.DiffDriveKinematics(0.08, 0.5), 90.0)
    _drive(odo, (3, 3), 100)
    x = odo.x
    # Board power-cycled: counters back at 0 while standing still.
    assert not odo.update((0, 0), (0, 0), 3.0)
    assert odo.update((0, 0), (0, 0), 3.02)
    assert odo.x == pytest.approx(x)


def test_odometry_baseline_reset_covers_small_counters():
    odo = hb.WheelOdometry(hb.DiffDriveKinematics(0.08, 0.5), 90.0)
    _drive(odo, (1, 1), 20)                       # only 20 ticks: below the jump threshold
    x = odo.x
    odo.reset_baseline()                          # the bridge does this while the board is silent
    assert odo.update((0, 0), (0, 0), 5.0)
    assert odo.update((2, 2), (0, 0), 5.02)
    assert odo.x == pytest.approx(x + 2 * 2 * math.pi * 0.08 / 90.0)


def test_indication_priorities():
    idle, moving, paused, fault = hb.LIGHT_IDLE, hb.LIGHT_MOVING, hb.LIGHT_PAUSED, hb.LIGHT_FAULT
    off, slow, fast = hb.BUZZER_OFF, hb.BUZZER_SLOW, hb.BUZZER_FAST
    assert hb.choose_indication(idle, off, moving=False, fault=False) == (idle, off)
    assert hb.choose_indication(idle, off, moving=True, fault=False) == (moving, off)
    assert hb.choose_indication(paused, off, moving=True, fault=False) == (moving, off)
    assert hb.choose_indication(paused, off, moving=False, fault=False) == (paused, off)
    assert hb.choose_indication(moving, fast, moving=True, fault=True) == (fault, fast)
    assert hb.choose_indication(fault, off, moving=True, fault=False) == (fault, off)
    assert hb.choose_indication(idle, off, True, False, travel_buzzer=True) == (moving, slow)
    assert hb.choose_indication(moving, fast, True, False, travel_buzzer=True) == (moving, fast)


def test_protocol_matches_the_dbc():
    cantools = pytest.importorskip("cantools")
    dbc = os.path.join(os.path.dirname(__file__), *[".."] * 4,
                       "Hoverboard_Firmware", "docs", "hoverboard_can.dbc")
    if not os.path.exists(dbc):
        pytest.skip("DBC file not found (package installed outside the repository)")
    db = cantools.database.load_file(dbc)

    assert db.get_message_by_name("ESTOP").frame_id == hb.ESTOP_ID
    assert db.get_message_by_name("SAFETY_STATUS").frame_id == hb.SAFETY_STATUS_ID
    assert db.get_message_by_name("BODY_CMD").frame_id == hb.BODY_CMD_ID
    assert db.get_message_by_name("HB_CMD").frame_id == hb.cmd_id(1)

    cmd = db.decode_message(hb.cmd_id(1), hb.encode_cmd(120, -45))
    assert (cmd["cmd_left"], cmd["cmd_right"]) == (120, -45)

    body = db.decode_message(hb.BODY_CMD_ID, hb.encode_body_cmd(hb.LIGHT_PAUSED, hb.BUZZER_FAST))
    assert (str(body["light_mode"]), str(body["buzzer"])) == ("paused", "fast")

    raw = db.encode_message("HB_STATUS", {
        "battery_voltage": 38.5, "battery_percent": 64, "board_temp": 31, "err_left": 0,
        "err_right": 4, "motors_enabled": 1, "cmd_timeout": 0, "safe_state": 0, "estop_active": 1,
        "aux_input": 0, "battery_low": 0, "temp_warning": 0, "can_error_counter": 3, "heartbeat": 9})
    s = hb.decode_status(raw)
    assert s.battery_voltage == pytest.approx(38.5)
    assert (s.battery_percent, s.board_temperature, s.error_right) == (64, 31, 4)
    assert s.motors_enabled and s.estop_active and not s.battery_low

    raw = db.encode_message("SAFETY_STATUS", {
        "estop_active": 1, "hoverboard_power": 0, "button_pressed": 1,
        "estop_cause": "button", "heartbeat": 17})
    safety = hb.decode_safety_status(raw)
    assert safety.estop_active and safety.button_pressed and not safety.hoverboard_power
    assert safety.cause == hb.CAUSE_BUTTON and safety.heartbeat == 17


def test_lift_cmd_payload():
    assert hb.encode_lift_cmd(hb.LIFT_GOTO, 0.05) == struct.pack("<BH", 1, 500)
    assert hb.encode_lift_cmd(hb.LIFT_HOME) == b"\x02\x00\x00"
    assert hb.encode_lift_cmd(hb.LIFT_GOTO, -1.0) == b"\x01\x00\x00"


def test_decode_lift_status():
    s = hb.decode_lift_status(struct.pack("<BBhhBB", hb.LIFT_IDLE, 0x01 | 0x02 | 0x08, 500, -12, 0, 9))
    assert s.state == hb.LIFT_IDLE and s.homed and s.at_target and s.limit_b and not s.limit_a
    assert s.position == pytest.approx(0.05)
    assert s.skew == pytest.approx(-0.0012)
    assert (s.fault, s.heartbeat) == (0, 9)


def test_lift_frames_match_the_dbc():
    cantools = pytest.importorskip("cantools")
    dbc = os.path.join(os.path.dirname(__file__), *[".."] * 4,
                       "Hoverboard_Firmware", "docs", "hoverboard_can.dbc")
    if not os.path.exists(dbc):
        pytest.skip("DBC file not found (package installed outside the repository)")
    db = cantools.database.load_file(dbc)
    cmd = db.decode_message(hb.LIFT_CMD_ID, hb.encode_lift_cmd(hb.LIFT_GOTO, 0.0375))
    assert str(cmd["command"]) == "goto" and cmd["target"] == pytest.approx(37.5)
    raw = db.encode_message("LIFT_STATUS", {
        "state": "moving", "homed": 1, "at_target": 0, "limit_a": 1, "limit_b": 0,
        "position": 21.4, "skew": -0.3, "fault": "none", "heartbeat": 5})
    assert db.get_message_by_name("LIFT_STATUS").frame_id == hb.LIFT_STATUS_ID
    s = hb.decode_lift_status(raw)
    assert s.state == hb.LIFT_MOVING and s.homed and s.limit_a and not s.at_target
    assert s.position == pytest.approx(0.0214) and s.skew == pytest.approx(-0.0003)
