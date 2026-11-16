# Real robot hardware (Raspberry Pi 5)

| Component | Connection | ROS driver | ROS interface |
|---|---|---|---|
| 2x RPLidar A1M8 (front, rear) | USB serial, 115200 baud | `rplidar_ros/rplidar_node` (`/front_lidar/rplidar`, `/rear_lidar/rplidar`) | `/scan`, `/scan_rear` |
| MPU9250 (+ AK8963 compass) | Pi I2C bus 1, address 0x68 | `amr_hardware/mpu9250_driver` + `imu_filter_madgwick` | `/imu/data_raw`, `/imu/mag`, `/imu/temperature` → `/imu/data` |
| 1x hoverboard mainboard: two hub motors on the centre axle (differential drive, 4 corner casters) | CAN, SocketCAN `can0` | `amr_hardware/can_bridge` | `/cmd_vel` in; `/odom`, `/joint_states`, `/hoverboard/status` out |
| STM32F411CEU6 body controller: e-stop authority, relay that switches the hoverboard board on | same CAN bus | `amr_hardware/can_bridge` | `/estop` service, `/estop/status` |
| Lift (two motors with encoders), stack light, buzzer | same STM32F411 | `amr_hardware/can_bridge` | `/lift_joint/cmd_pos` in, `/lift/status` out, `/lift/home` service; `/indicator` in |

All drivers are started by `amr_bringup/bringup.launch.py` (and therefore by mapping and navigation)
through `amr_hardware/launch/hardware.launch.py`. Every tunable lives in
`src/amr_hardware/config/hardware.yaml`.

Wiring, pinouts, the cables between the boxes and the safety chain are in
[ELECTRICAL.md](../../ELECTRICAL.md). This page covers the software side of the same hardware.

## 1. Operating system

1. Flash **Ubuntu 24.04 LTS Server (64-bit)** for Raspberry Pi and install **ROS 2 Jazzy**
   (`ros-jazzy-ros-base` is enough; no desktop needed).
2. Clone the repository and install the dependencies (no Gazebo/RViz on the robot):

   ```bash
   cd RaspberryPi_ROS2
   bash install_dependencies.sh
   ```

   The script also adds your user to `dialout` (lidars) and `i2c` (IMU). Log out and back in.
3. Build: `colcon build --symlink-install && source install/setup.bash`.

## 2. CAN bus

### Interface on the Pi

Run the setup script once and reboot:

```bash
sudo bash scripts/setup_pi_hardware.sh                                        # MCP2515, 16 MHz crystal, 1 Mbit/s
sudo bash scripts/setup_pi_hardware.sh --oscillator 8000000 --bitrate 500000  # 8 MHz crystal
sudo bash scripts/setup_pi_hardware.sh --usb                                  # USB adapter (candleLight / gs_usb)
```

It enables SPI, the `mcp2515-can0` overlay (INT on GPIO25) and I2C in `/boot/firmware/config.txt`,
raises the USB current limit for the lidars, and installs `can0.service`, which brings `can0` up at
boot.

The service sets the bit timing explicitly instead of only a bitrate. The hoverboard board runs
from an internal RC oscillator, and the kernel's default timing (synchronisation jump width of one
time quantum) does not tolerate its drift. The script uses the same timing the body controller
computes for the same crystal.

The hoverboard firmware runs at **1 Mbit/s** (`CAN_BITRATE` in `Hoverboard_Firmware/Inc/config.h`).
An MCP2515 with an 8 MHz crystal cannot reach 1 Mbit/s: use 500 kbit/s on every node instead.

```bash
ip -details link show can0    # state ERROR-ACTIVE and the timing
candump can0                  # from can-utils
```

Wiring: CAN_H/CAN_L twisted pair, shared GND, **120 Ω termination only at the two physical ends**
of the bus, which are the hoverboard PCB and the body controller. The Pi is in the middle.

### Hoverboard mainboard

The robot is a **differential drive**: one hoverboard mainboard flashed with `VARIANT_CAN` (see
[Hoverboard_Firmware](../../Hoverboard_Firmware/README.md)) drives the two hub motors on the centre
axle; four passive casters at the corners carry the rest of the load.

| Board | `CAN_NODE_ID` | Left motor | Right motor | Frames it sends |
|---|---|---|---|---|
| drive | `1` | left wheel | right wheel | 0x181 odom, 0x281 motion, 0x381 status |

Change `board_id` in `hardware.yaml` if you flash another ID. The firmware convention is
**positive = robot forward** for both wheels; if a wheel turns the wrong way, set
`INVERT_L_DIRECTION` / `INVERT_R_DIRECTION` in `config.h` (this flips the command and the feedback
together). Disable `DEBUG_SERIAL_USART3` in production builds (it adds loop jitter).

The e-stop relay takes the place of the board's power button: its contact goes on the ON SWITCH
connector and switches the board's logic supply, not the battery line. `POWER_SWITCH_EXTERNAL` in
`config.h` (on by default for `VARIANT_CAN`) makes the firmware follow that contact
([ELECTRICAL.md](../../ELECTRICAL.md#8-hoverboard-mainboard-and-the-hb-pcb)).

### E-stop controller (STM32F411CEU6, `BODY` node)

Firmware, wiring and the build/upload tool:
[STM32_CANbus_Embedded/Blackpill_CAN_Software](../../STM32_CANbus_Embedded/Blackpill_CAN_Software/README.md)
(Blackpill + MCP2515; an 8 MHz MCP2515 crystal limits the whole bus to 500 kbit/s).

The STM32F411 is the e-stop authority. It switches the relay that powers the hoverboard board and
has the physical e-stop button. Its required behaviour (documented in the DBC):

1. **Trip** on any of: `ESTOP = 1` from the Pi, its button, or no `HB_CMD` frame (0x201) for
   **500 ms** (Pi / bridge crashed; the bridge sends `HB_CMD` at 50 Hz, also when stopped). When the
   button trips it, it broadcasts `ESTOP = 1` itself.
2. **Brake, then cut power**: the hoverboard brakes actively on `ESTOP = 1`. Only once the wheels
   are stopped (or after a short timeout) does the controller open the relay, because an unpowered
   board lets the motors coast.
3. **Release** only on `ESTOP = 0` from the Pi, and **never while the button is pressed**. It then
   closes the relay; the hoverboard reboots and its tick counters restart at 0 (the bridge
   re-baselines its odometry).
4. Report its state in `SAFETY_STATUS` (0x011) at 10 Hz and immediately on change.

Protocol (from [hoverboard_can.dbc](../../Hoverboard_Firmware/docs/hoverboard_can.dbc), little-endian, n = hoverboard node id):

| ID | Direction | Content | Rate |
|---|---|---|---|
| 0x010 | Pi (or BODY button) → all | e-stop: `u8` 1 = latch, 0 = release | on request |
| 0x011 | BODY → Pi | byte 0: bit0 e-stop active, bit1 hoverboard power (relay closed), bit2 button pressed, bits 4-6 cause (0 none, 1 host request, 2 button, 3 host timeout); byte 1: heartbeat | 10 Hz + on change |
| 0x200+n | Pi → board | `i16` left rpm, `i16` right rpm | 50 Hz (bridge) |
| 0x180+n | board → Pi | `i32` left/right hall ticks (~90 per wheel revolution) | 50 Hz |
| 0x280+n | board → Pi | `i16` left/right rpm, `i16` left/right current [A×100] | 50 Hz |
| 0x380+n | board → Pi | battery V×100, battery %, temperature, error codes, status flags, CAN error counter, heartbeat | 10 Hz |

The same node also takes `BODY_CMD` (0x020, stack light and buzzer) and `LIFT_CMD` (0x021), and
sends `LIFT_STATUS` (0x022). See section 5.

### What the CAN bridge does

- Converts `/cmd_vel` (`linear.x`, `angular.z`; `linear.y` is ignored) to left/right wheel speeds
  and sends them at 50 Hz. Without a `/cmd_vel` message for `cmd_vel_timeout` (0.5 s) it commands
  0 rpm. Both wheels are scaled together to stay under `max_wheel_rpm` (150 rpm ≈ 1.3 m/s) so the
  curvature is kept.
- While an e-stop is requested or `SAFETY_STATUS` reports it active, it commands 0 rpm, so a release
  never resumes an old command.
- Integrates the hall ticks into `/odom` (pose) and reports the measured wheel rpm as the twist
  (the EKF fuses only the twist). It publishes no TF: the EKF owns `odom → base_footprint`.
  Tick jumps (board restarted) are dropped and the odometry re-baselined. While the board is
  silent (power cut by the e-stop) the pose is held and the twist is 0.
- When the stack starts and the controller is latched only because nobody was sending commands
  (cause "host timeout"), the bridge releases it (`release_host_timeout_on_start`). A pressed
  button or a requested e-stop stays latched.
- Publishes `/joint_states` for `left_wheel_joint` and `right_wheel_joint` so the wheels turn in RViz.
- Publishes every decoded hoverboard field on `/hoverboard/status`
  (`amr_interfaces/msg/HoverboardStatus`) and the e-stop controller state on `/estop/status`
  (`amr_interfaces/msg/SafetyStatus`) at 10 Hz, with `online: false` when a node stops reporting.
  E-stop changes and their cause, an offline board (unless the relay is open), hall/stall errors,
  low battery and over-temperature are also logged.
- Serves `/estop` (`std_srvs/srv/SetBool`): `true` latches the e-stop on every CAN node, `false`
  requests the release (refused while the button is pressed).

```bash
ros2 service call /estop std_srvs/srv/SetBool "{data: true}"    # stop everything
ros2 service call /estop std_srvs/srv/SetBool "{data: false}"   # release
ros2 topic echo /estop/status                                   # who stopped the robot and why
```

Without the STM32F411 on the bus the bridge warns every 30 s and the hoverboard's own e-stop
(active braking, no power cut) is the only one.

### Parameters to measure

`wheel_radius` and `wheel_separation` (distance between the two drive-wheel contact points) are
placeholders until the final CAD/URDF exists and **must be measured on the real robot**. Set them
in **both** `amr_hardware/config/hardware.yaml` and
`amr_description/urdf/urdf/drivetrain.urdf.xacro` (`drive_wheel_radius`, `drive_wheel_separation`;
the caster positions live there too). Check them by driving a known distance and a full turn and
comparing `/odom`: a wrong distance means a wrong radius, a wrong angle means a wrong separation.

## 3. Lidars (RPLidar A1M8)

- **Ports**: `ttyUSB0/1` depend on plug-in order. Use the stable paths from
  `ls -l /dev/serial/by-path/` as `serial_port` in `hardware.yaml`.
- **Mounting**: the URDF places the front lidar at (0.29, 0, 0.15) m from `base_link` and the rear
  one at (-0.29, 0, 0.15) m rotated 180°. Measure the real positions and update the two
  `lidar_sensor` blocks in `amr_description/urdf/urdf/robot.urdf.xacro`.
- **Orientation check**: open RViz on a laptop (fixed frame `base_link`) and put an object in front
  of the robot. If one lidar shows it behind the robot, set `flip_x_axis: true` for that lidar; if
  the scan is mirrored, set `inverted: true`.
- **Field of view**: the A1M8 sees 360°, but the half facing the robot body would hit the chassis and
  lift. `amr_mapping/config/laser_filters.yaml` crops both scans to ±125° (same as the simulated
  sensors). Adjust `lower_angle`/`upper_angle` if the real body blocks more or less.
- Motor control services: `/front_lidar/stop_motor`, `/front_lidar/start_motor` (same for `rear_lidar`).

## 4. IMU (MPU9250)

- **Wiring** (3.3 V part): VCC → pin 1 (3.3 V), GND → pin 6, SDA → pin 3, SCL → pin 5.
  `sudo i2cdetect -y 1` must show `68` (and `0c`, the magnetometer, once the driver has run).
- **Mounting**: x forward, y left, z up. Otherwise update the `imu_sensor` block (`xyz_offset`,
  `rpy_offset`) in `robot.urdf.xacro`.
- **Start-up**: keep the robot still for ~2 s while the gyro bias is measured. Re-measure at any time
  (robot still):

  ```bash
  ros2 service call /imu/calibrate_gyro std_srvs/srv/Trigger
  ```

- **Orientation**: `imu_filter_madgwick` turns `/imu/data_raw` into `/imu/data` (orientation +
  rates), which the EKF fuses. `publish_tf` is off: the EKF owns the odometry transform.
- **Compass**: `/imu/mag` is always published (in the accel/gyro axes, Tesla) but **not fused by
  default** (`use_mag: false`): the hub-motor magnets and steel racks distort it. To use it, do a
  hard-iron calibration first: rotate the robot a full turn while recording `/imu/mag`, set
  `mag_bias_x/y/z` in `hardware.yaml` to `(max + min) / 2` of each axis, then set `use_mag: true`.
- Boards that report `WHO_AM_I = 0x70` are MPU6500 clones without a magnetometer: the driver warns
  and runs without `/imu/mag`.

## 5. Lift, lights and buzzer

The body controller drives all three. Wiring and bring-up: [ELECTRICAL.md](../../ELECTRICAL.md),
sections 6 and 7.

**Lift.** The bridge forwards `/lift_joint/cmd_pos` (`std_msgs/Float64`, metres above the home
position) as `LIFT_CMD` and publishes the controller's `LIFT_STATUS` on `/lift/status`
(`amr_interfaces/msg/LiftStatus`). The measured height is also published as `lift_joint` in
`/joint_states`, so the lift moves in RViz. The first command after power-up homes the lift on its
limit switches before moving.

```bash
ros2 topic pub --once /lift_joint/cmd_pos std_msgs/msg/Float64 "{data: 0.05}"   # up 5 cm
ros2 topic echo /lift/status --once
ros2 service call /lift/home std_srvs/srv/Trigger                               # home; also clears a lift fault
```

During a transport the mission manager waits for the lift to report that it has arrived. A lift
fault or a timeout (`lift_timeout`) fails the task instead of driving off with a rack half lifted.
An e-stop drops the lift move, and it does not resume on release.

**Stack light and buzzer.** The mission manager publishes `/indicator`
(`amr_interfaces/msg/Indicator`). The bridge shows "moving" by itself whenever the wheels are
commanded and "fault" on a drive or lift fault. `travel_buzzer: true` in `hardware.yaml` adds a
beep every second while moving. A latched e-stop is shown by the controller without the Pi.

## 6. First-drive checklist

1. Wheels off the ground, e-stop within reach.
2. `candump can0` shows 0x181/0x281/0x381 from the hoverboard and 0x011 from the e-stop controller.
3. `ros2 launch amr_bringup bringup.launch.py` — no errors from `can_bridge`, `mpu9250`, both lidars.
4. `ros2 topic echo /hoverboard/status --once`: `online: true`, plausible battery.
   `ros2 topic echo /estop/status --once`: `online: true`, `hoverboard_power: true`.
5. E-stop test: press the button → wheels brake, then the relay opens (`hoverboard_power: false`,
   cause button). Release the button and call `/estop` with `false` → relay closes, board online
   again. Repeat with `/estop` `true`, and by killing the bridge (cause host timeout).
6. `ros2 topic hz /scan /scan_rear /imu/data /odom /odometry/filtered`.
7. Direction test, one at a time:

   ```bash
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"     # both wheels forward
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"    # left back, right forward
   ```

   On the floor, `angular.z > 0` must turn the robot **counter-clockwise** seen from above. If it
   turns clockwise, the left and right motors are swapped: swap their connectors (or their wiring).
8. Calibrate `wheel_radius`, `wheel_separation`, the lidar poses and the lidar orientation.
9. Map the site ([launch.md](launch.md#map-a-new-site)), then re-tune the docking parameters
   ([mission_control.md](mission_control.md#tuning-on-the-real-robot)).
