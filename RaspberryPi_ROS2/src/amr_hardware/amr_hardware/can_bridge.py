"""ROS 2 <-> CAN bridge for the AMR (SocketCAN via python-can).

Owns the robot's CAN bus (one process for every CAN device):
  * hoverboard mainboard (Hoverboard_Firmware VARIANT_CAN, node id board_id): the two
    hub motors of the differential drive on the centre axle;
  * STM32F411 body controller: e-stop authority (its relay switches the hoverboard
    board's power), stack light and buzzer. The lift frames belong in this node too.

Subscribes:  cmd_vel            geometry_msgs/Twist (linear.x, angular.z; linear.y is ignored)
             indicator          amr_interfaces/Indicator (stack light + buzzer request)
             lift_joint/cmd_pos std_msgs/Float64 (lift height above home [m])
Publishes:   odom               nav_msgs/Odometry (wheel odometry, no TF)
             joint_states       sensor_msgs/JointState (left/right wheel, lift_joint once the lift reports)
             hoverboard/status  amr_interfaces/HoverboardStatus
             estop/status       amr_interfaces/SafetyStatus
             lift/status        amr_interfaces/LiftStatus
Services:    estop              std_srvs/SetBool (true = latch e-stop, false = request release)
             lift/home          std_srvs/Trigger (home the lift on its limit switches; clears a lift fault)
"""

import math
import struct
import threading
import time

import can
import rclpy
from amr_interfaces.msg import HoverboardStatus, Indicator, LiftStatus, SafetyStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger

from amr_hardware import hoverboard as hb

JOINTS = ("left_wheel_joint", "right_wheel_joint")
LIFT_JOINT = "lift_joint"
CAUSES = {
    hb.CAUSE_NONE: "none",
    hb.CAUSE_HOST_REQUEST: "host request",
    hb.CAUSE_BUTTON: "e-stop button",
    hb.CAUSE_HOST_TIMEOUT: "host timeout",
}
MOVING_RPM = 0.5          # commanded wheel speed above this shows "moving" on the stack light


def _diagonal(values):
    cov = [0.0] * 36
    for i, v in enumerate(values):
        cov[i * 7] = v
    return cov


# x, y, z, roll, pitch, yaw. The EKF fuses only vx, vy (always 0) and vyaw from this message.
POSE_COVARIANCE = _diagonal([0.01, 0.01, 1e3, 1e3, 1e3, 0.05])
TWIST_COVARIANCE = _diagonal([0.0025, 0.0025, 1e3, 1e3, 1e3, 0.01])


class Board:
    """Latest data received from the hoverboard mainboard (with arrival times)."""

    def __init__(self, node_id):
        self.node_id = node_id
        self.ticks = None
        self.ticks_time = None
        self.rpm = (0, 0)
        self.current = (0.0, 0.0)
        self.motion_time = None
        self.status = None
        self.status_time = None


class CanBridge(Node):
    def __init__(self):
        super().__init__("can_bridge")

        def param(name, default):
            return self.declare_parameter(name, default).value

        interface = param("can_interface", "can0")
        self.board = Board(param("board_id", 1))
        self.kin = hb.DiffDriveKinematics(
            param("wheel_radius", 0.0825),
            param("wheel_separation", 0.55),
        )
        self.odometry = hb.WheelOdometry(self.kin, param("ticks_per_revolution", 90.0))
        self.max_rpm = param("max_wheel_rpm", 150.0)
        self.cmd_timeout = param("cmd_vel_timeout", 0.5)
        self.status_timeout = param("status_timeout", 0.5)
        self.feedback_timeout = param("feedback_timeout", 0.25)
        self.indicator_timeout = param("indicator_timeout", 1.0)
        self.travel_buzzer = param("travel_buzzer", False)
        self.release_host_timeout = param("release_host_timeout_on_start", True)
        self.odom_frame = param("odom_frame", "odom")
        self.base_frame = param("base_frame", "base_footprint")
        rate = param("rate", 50.0)
        status_rate = param("status_rate", 10.0)

        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0)
        self._cmd_time = -math.inf
        self._estop_requested = False
        self._safety = None
        self._safety_time = None
        self._indicator = (hb.LIGHT_IDLE, hb.BUZZER_OFF)
        self._indicator_time = -math.inf
        self._moving = False
        self._board_online = False
        self._estop_logged = None
        self._safety_was_active = False
        self._startup_checked = False
        self._lift = None
        self._lift_time = None
        self._lift_target = None           # last commanded height [m], re-sent at 5 Hz
        self._lift_fault_logged = False

        node_id = self.board.node_id
        self._frames = {
            hb.odom_id(node_id): "odom",
            hb.motion_id(node_id): "motion",
            hb.status_id(node_id): "status",
            hb.SAFETY_STATUS_ID: "safety",
            hb.LIFT_STATUS_ID: "lift",
        }
        self.bus = can.Bus(
            interface="socketcan",
            channel=interface,
            can_filters=[{"can_id": i, "can_mask": 0x7FF, "extended": False} for i in self._frames],
        )

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.board_pub = self.create_publisher(HoverboardStatus, "hoverboard/status", 10)
        self.safety_pub = self.create_publisher(SafetyStatus, "estop/status", 10)
        self.lift_pub = self.create_publisher(LiftStatus, "lift/status", 10)
        self.create_subscription(Float64, "lift_joint/cmd_pos", self._on_lift_cmd, 10)
        self.create_service(Trigger, "lift/home", self._on_lift_home)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Indicator, "indicator", self._on_indicator, 10)
        self.create_service(SetBool, "estop", self._on_estop)
        self.create_timer(1.0 / rate, self._step)
        self.create_timer(1.0 / status_rate, self._publish_status)
        self.create_timer(0.2, self._send_indication)
        self._notifier = can.Notifier(self.bus, [self._on_frame])

        self.get_logger().info(
            f"CAN bridge on {interface}: hoverboard id {node_id}, "
            f"wheel limit {self.max_rpm:.0f} rpm")

    # ── CAN receive (python-can notifier thread) ─────────────────────
    def _on_frame(self, msg):
        kind = self._frames.get(msg.arbitration_id)
        if kind is None or msg.is_extended_id or msg.is_remote_frame or msg.is_error_frame:
            return
        try:
            if kind == "odom":
                value = hb.decode_odom(msg.data)
            elif kind == "motion":
                value = hb.decode_motion(msg.data)
            elif kind == "status":
                value = hb.decode_status(msg.data)
            elif kind == "lift":
                value = hb.decode_lift_status(msg.data)
            else:
                value = hb.decode_safety_status(msg.data)
        except struct.error:
            return
        now = time.monotonic()
        with self._lock:
            if kind == "odom":
                self.board.ticks = value
                self.board.ticks_time = now
            elif kind == "motion":
                self.board.rpm = value[:2]
                self.board.current = value[2:]
                self.board.motion_time = now
            elif kind == "status":
                self.board.status = value
                self.board.status_time = now
            elif kind == "lift":
                self._lift = value
                self._lift_time = now
            else:
                self._safety = value
                self._safety_time = now

    def _send(self, arbitration_id, data):
        try:
            self.bus.send(
                can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False),
                timeout=0.01)
            return True
        except can.CanError as e:
            self.get_logger().warning(f"CAN send failed: {e}", throttle_duration_sec=2.0)
            return False

    def _safety_online(self, now):
        return self._safety_time is not None and now - self._safety_time < self.status_timeout

    @staticmethod
    def _fresh(stamp, now, timeout):
        return stamp is not None and now - stamp < timeout

    # ── ROS callbacks ────────────────────────────────────────────────
    def _on_cmd_vel(self, msg):
        with self._lock:
            self._cmd = (msg.linear.x, msg.angular.z)
            self._cmd_time = time.monotonic()

    def _on_indicator(self, msg):
        with self._lock:
            self._indicator = (msg.light, msg.buzzer)
            self._indicator_time = time.monotonic()

    def _on_lift_cmd(self, msg):
        if not math.isfinite(msg.data):
            return
        height = max(0.0, msg.data)
        with self._lock:
            self._lift_target = height
        self._send(hb.LIFT_CMD_ID, hb.encode_lift_cmd(hb.LIFT_GOTO, height))

    def _on_lift_home(self, request, response):
        with self._lock:
            self._lift_target = None       # stays at home until the next command
        response.success = self._send(hb.LIFT_CMD_ID, hb.encode_lift_cmd(hb.LIFT_HOME))
        response.message = "Lift homing" if response.success else "CAN send failed"
        return response

    def _on_estop(self, request, response):
        now = time.monotonic()
        with self._lock:
            self._estop_requested = request.data
            button = self._safety_online(now) and self._safety.button_pressed
        if not request.data and button:
            response.success = False
            response.message = "Release refused: the e-stop button is pressed"
            self.get_logger().warning(response.message)
            return response
        response.success = self._send(hb.ESTOP_ID, hb.encode_estop(request.data))
        if not response.success:
            response.message = "CAN send failed"
        elif request.data:
            response.message = "E-stop latched: hoverboard braking, controller cutting its power"
            self.get_logger().warning(response.message)
        else:
            response.message = "E-stop release requested"
            self.get_logger().info(response.message)
        return response

    def _step(self):
        now = time.monotonic()
        b = self.board
        with self._lock:
            safety_stop = self._safety_online(now) and self._safety.estop_active
            fresh = now - self._cmd_time < self.cmd_timeout
            # While e-stopped the wheels get 0 rpm, so release never resumes a stale command.
            v, wz = self._cmd if fresh and not (self._estop_requested or safety_stop) else (0.0, 0.0)
            ticks = b.ticks if self._fresh(b.ticks_time, now, self.feedback_timeout) else None
            # A board that stopped reporting (power cut) must not leave its last speed in /odom.
            rpm = b.rpm if self._fresh(b.motion_time, now, self.feedback_timeout) else (0, 0)

        self._send_wheel_command(v, wz)
        self._publish_odometry(ticks, rpm, now)

    def _send_wheel_command(self, v, wz):
        rpm = [hb.rad_s_to_rpm(w) for w in self.kin.inverse(v, wz)]
        peak = max(abs(r) for r in rpm)
        if peak > self.max_rpm:
            # Scale both wheels together so the curvature is kept.
            rpm = [r * self.max_rpm / peak for r in rpm]
        self._moving = max(abs(r) for r in rpm) > MOVING_RPM
        self._send(hb.cmd_id(self.board.node_id), hb.encode_cmd(rpm[0], rpm[1]))

    def _publish_odometry(self, ticks, rpm, now):
        odo = self.odometry
        if ticks is None:
            # Board silent (unpowered by the e-stop relay, or unplugged): its counters restart
            # at 0, so the next sample only sets the baseline. The pose is held, the twist is 0.
            odo.reset_baseline()
        elif not odo.update(ticks, rpm, now):
            self.get_logger().warning(
                "Hall tick jump (board restarted?), re-baselining odometry",
                throttle_duration_sec=2.0)

        wheel_speeds = [hb.rpm_to_rad_s(r) for r in rpm]
        v, wz = self.kin.forward(*wheel_speeds)
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = odo.x
        odom.pose.pose.position.y = odo.y
        odom.pose.pose.orientation.z = math.sin(odo.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(odo.yaw / 2.0)
        odom.pose.covariance = POSE_COVARIANCE
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = TWIST_COVARIANCE
        self.odom_pub.publish(odom)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = list(JOINTS)
        joints.position = list(odo.wheel_angle)
        joints.velocity = wheel_speeds
        with self._lock:
            lift = self._lift if self._fresh(self._lift_time, now, self.status_timeout) else None
        if lift is not None:
            joints.name.append(LIFT_JOINT)
            joints.position.append(lift.position)
            joints.velocity.append(0.0)
        self.joint_pub.publish(joints)

    def _send_indication(self):
        """BODY_CMD at 5 Hz: stack light and buzzer."""
        now = time.monotonic()
        b = self.board
        with self._lock:
            light, buzzer = (self._indicator if now - self._indicator_time < self.indicator_timeout
                             else (hb.LIGHT_IDLE, hb.BUZZER_OFF))
            status = b.status if self._fresh(b.status_time, now, self.status_timeout) else None
            fault = status is not None and bool(status.error_left or status.error_right)
            lift = self._lift if self._fresh(self._lift_time, now, self.status_timeout) else None
            lift_target = self._lift_target
        fault = fault or (lift is not None and lift.state == hb.LIFT_FAULT)
        light, buzzer = hb.choose_indication(light, buzzer, self._moving, fault, self.travel_buzzer)
        self._send(hb.BODY_CMD_ID, hb.encode_body_cmd(light, buzzer))
        # Repeat the lift target so a lost frame or a restarted controller is caught up.
        # Not while it reports a fault (it must be homed first) or the e-stop is latched.
        if self._safety_was_active or self._estop_requested:
            # The controller dropped the move. Forget the target, so the lift does not
            # start by itself when the e-stop is released.
            with self._lock:
                self._lift_target = None
        elif lift_target is not None and lift is not None and lift.state not in (hb.LIFT_FAULT, hb.LIFT_HOMING):
            self._send(hb.LIFT_CMD_ID, hb.encode_lift_cmd(hb.LIFT_GOTO, lift_target))

    def _publish_status(self):
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        b = self.board
        with self._lock:
            status, status_time = b.status, b.status_time
            rpm, current, ticks = b.rpm, b.current, b.ticks
            if not self._fresh(b.motion_time, now, self.feedback_timeout):
                rpm, current = (0, 0), (0.0, 0.0)
            safety, safety_online = self._safety, self._safety_online(now)
            if safety_online:
                # Released by the controller (GUI, its console): drop our own request too.
                if self._safety_was_active and not safety.estop_active:
                    self._estop_requested = False
                self._safety_was_active = safety.estop_active
            estop_requested = self._estop_requested

        # ── e-stop controller ────────────────────────────────────────
        msg = SafetyStatus()
        msg.header.stamp = stamp
        msg.online = safety_online
        if safety is not None:
            msg.estop_active = safety.estop_active
            msg.hoverboard_power = safety.hoverboard_power
            msg.button_pressed = safety.button_pressed
            msg.cause = safety.cause
            msg.heartbeat = safety.heartbeat
        self.safety_pub.publish(msg)

        if safety_online and not self._startup_checked:
            self._startup_checked = True
            # The controller trips its host watchdog whenever this bridge is not running.
            # Starting the stack is the operator's deliberate act, so clear that latch;
            # a button or a requested e-stop stays latched.
            if (self.release_host_timeout and safety.estop_active
                    and safety.cause == hb.CAUSE_HOST_TIMEOUT and not safety.button_pressed):
                if self._send(hb.ESTOP_ID, hb.encode_estop(False)):
                    self.get_logger().info(
                        "Controller was latched by its host watchdog (stack was not running): released")

        if not safety_online:
            self.get_logger().warning(
                "No status from the e-stop controller (STM32F411, frame 0x011): "
                "only the hoverboard's own e-stop is available", throttle_duration_sec=30.0)
        estop = (safety.estop_active if safety_online else estop_requested)
        if estop != self._estop_logged:
            if estop:
                cause = CAUSES.get(safety.cause, "?") if safety_online else "host request"
                self.get_logger().warning(f"E-STOP ACTIVE (cause: {cause})")
            elif self._estop_logged is not None:
                self.get_logger().info("E-stop released")
            self._estop_logged = estop

        # ── lift ─────────────────────────────────────────────────────
        with self._lock:
            lift, lift_online = self._lift, self._fresh(self._lift_time, now, self.status_timeout)
        lmsg = LiftStatus()
        lmsg.header.stamp = stamp
        lmsg.online = lift_online
        if lift is not None:
            lmsg.state = lift.state
            lmsg.fault = lift.fault
            lmsg.homed = lift.homed
            lmsg.at_target = lift.at_target
            lmsg.limit_a = lift.limit_a
            lmsg.limit_b = lift.limit_b
            lmsg.position = lift.position
            lmsg.skew = lift.skew
            lmsg.heartbeat = lift.heartbeat
            faulted = lift_online and lift.state == hb.LIFT_FAULT
            if faulted and not self._lift_fault_logged:
                names = {1: "homing timeout", 2: "motor stalled or encoder not counting",
                         3: "left and right sides too far apart"}
                self.get_logger().error(
                    f"Lift fault: {names.get(lift.fault, lift.fault)}. Clear it with the lift/home service")
            self._lift_fault_logged = faulted
        self.lift_pub.publish(lmsg)

        # ── hoverboard ───────────────────────────────────────────────
        online = self._fresh(status_time, now, self.status_timeout)
        if online and not self._board_online:
            self.get_logger().info(f"Hoverboard (CAN id {b.node_id}) online")
        self._board_online = online
        unpowered = safety_online and not safety.hoverboard_power
        if not online and not unpowered:
            self.get_logger().warning(
                f"No status from the hoverboard (CAN id {b.node_id})", throttle_duration_sec=5.0)

        board = HoverboardStatus()
        board.header.stamp = stamp
        board.name = "drive"
        board.node_id = b.node_id
        board.online = online
        board.rpm_left, board.rpm_right = rpm
        board.current_left, board.current_right = current
        if ticks is not None:
            board.ticks_left, board.ticks_right = ticks
        if status is not None:
            board.battery_voltage = status.battery_voltage
            board.battery_percent = status.battery_percent
            board.board_temperature = status.board_temperature
            board.error_left = status.error_left
            board.error_right = status.error_right
            board.motors_enabled = status.motors_enabled
            board.cmd_timeout = status.cmd_timeout
            board.safe_state = status.safe_state
            board.estop_active = status.estop_active
            board.aux_input = status.aux_input
            board.battery_low = status.battery_low
            board.temp_warning = status.temp_warning
            board.can_error_counter = status.can_error_counter
            board.heartbeat = status.heartbeat
            if online and (status.error_left or status.error_right
                           or status.battery_low or status.temp_warning):
                self.get_logger().warning(
                    f"Hoverboard faults: err L={status.error_left} R={status.error_right}"
                    f"{' battery-low' if status.battery_low else ''}"
                    f"{' over-temp' if status.temp_warning else ''}",
                    throttle_duration_sec=5.0)
        self.board_pub.publish(board)

    def destroy_node(self):
        self._notifier.stop()
        self._send(hb.cmd_id(self.board.node_id), hb.encode_cmd(0, 0))
        self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CanBridge()
    except (OSError, can.CanError) as e:
        rclpy.logging.get_logger("can_bridge").fatal(
            f"Cannot open the CAN interface: {e}. See docs/hardware.md (CAN bus).")
        rclpy.try_shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
