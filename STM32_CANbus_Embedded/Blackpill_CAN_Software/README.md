# Blackpill CAN Software: AMR body controller

Firmware for the **BODY** node of the LEVERAGE AMR: a WeAct Blackpill (STM32F411CEU6) with an
MCP2515 CAN controller. It is the robot's **e-stop authority**. It switches the relay that powers the
hoverboard motor board, reads the physical e-stop button, watches the Raspberry Pi, and reports its
state on the CAN bus. It also runs the lift (two motors with encoders, homed on limit switches),
the stack light and the buzzer.

The project builds with PlatformIO (Arduino / STM32duino). **Blackpill Tool.bat** opens a small desktop
tool that builds, uploads over **SWD** (ST-Link) or **USB DFU**, and has a serial terminal with a live
status panel.

![Blackpill CAN Tool: build, upload, terminal and live controller status (example values)](docs/blackpill-tool.png)

- [Hardware and wiring](#hardware-and-wiring)
- [Blackpill Tool](#blackpill-tool)
- [Building and uploading without the tool](#building-and-uploading-without-the-tool)
- [What the firmware does](#what-the-firmware-does)
- [Debug console](#debug-console)
- [CAN bitrate and the MCP2515 crystal](#can-bitrate-and-the-mcp2515-crystal)
- [Configuration](#configuration)
- [Testing](#testing)
- [Project layout](#project-layout)
- [Status](#status)

## Hardware and wiring

The full wiring, with the driver circuits and the cables to the other boxes, is in
[ELECTRICAL.md](../../ELECTRICAL.md). Board pinout: [../Docs/Blackpill_Pinout.jpg](../Docs/Blackpill_Pinout.jpg).

| Signal | Blackpill pin | Notes |
|---|---|---|
| MCP2515 SCK / MISO / MOSI | PA5 / PA6 / PA7 | SPI1, 8 MHz |
| MCP2515 CS | PA4 | |
| MCP2515 INT | PB0 | Reserved. The firmware polls over SPI |
| Relay (hoverboard power) | PB12 | HIGH = closed. **Fit a pull-down on the relay driver**, so the relay stays open while the MCU is in reset or being flashed |
| E-stop button | PB13 | Normally-closed contact to GND, internal pull-up. Pressed **or a broken wire** reads as pressed |
| Stack light red / amber / green | PB14 / PB15 / PA8 | HIGH = on, each through a low-side MOSFET |
| Buzzer | PB5 | HIGH = on |
| Lift PWM left / right | PA2 / PA3 | TIM9, 20 kHz, to the Cytron driver |
| Lift direction left / right | PB8 / PB9 | |
| Lift encoder left A / B | PB6 / PB7 | TIM4 |
| Lift encoder right A / B | PA15 / PB3 | TIM2 |
| Lift limit switch A / B | PB10 / PB1 | Normally-closed to GND. Actuated or a broken wire reads as actuated |
| Status LED | PC13 | On-board blue LED |
| USB-C | PA11 / PA12 | Debug console (USB CDC) and DFU uploads |
| SWD | PA13 (SWDIO), PA14 (SWCLK), GND, 3V3 | ST-Link uploads and debugging |

The lift pins are tied to their timers. Moving one means changing `src/lift_hw.cpp`, not only
`config.h`.

- **MCP2515 module supply.** Common blue modules run the MCP2515 and TJA1050 from 5 V. At 5 V the
  MCP2515's input-high threshold is above the Blackpill's 3.3 V outputs, which is out of spec even
  though it often works on the bench. For the PCB, run the MCP2515 from 3.3 V (it supports 2.7–5.5 V)
  with a 3.3 V transceiver such as the SN65HVD230, or level-shift SCK, MOSI and CS.
- **Termination.** Fit 120 Ω only at the two physical ends of the bus. This node is one of them.
- **LED.** It flickers fast when the CAN controller is missing, blinks at 2 Hz while the e-stop is
  latched, and gives a short heartbeat flash while running.

## Blackpill Tool

Double-click **`Blackpill Tool.bat`** or run `python tools/blackpill_gui.py`. The tool needs Python 3
with tkinter and PlatformIO, installed with the VS Code PlatformIO extension. The terminal uses
PlatformIO's own Python (pyserial). dfu-util and OpenOCD come with PlatformIO.

| Area | What it does |
|---|---|
| **Build** | Choose the CAN bitrate and the MCP2515 crystal, and see the resulting bit timing (unreachable combinations are refused). **Build** and **Clean**. Both settings are written to `platformio.ini`. |
| **Upload** | **SWD (ST-Link)** runs PlatformIO's upload. **USB DFU** builds, sends `dfu` to the running firmware so it reboots into the STM32 bootloader (no BOOT0 needed), waits for the bootloader, then flashes with dfu-util. **Check ST-Link** and **Find DFU device** test the connection. After an upload the terminal opens automatically. |
| **Build output** | Live, coloured output of every command, with hints for the usual problems: missing ST-Link or driver, WinUSB for DFU, a COM port in use, an impossible bitrate. |
| **Serial terminal** | Finds the controller by USB ID (or pick a port), reconnects after resets and uploads, keeps command history (↑/↓), and has one-click commands (`status`, `can`, `stats`, `log`, `selftest`, …). |
| **Controller panel** | Live state from the firmware's `@S` lines: e-stop and cause, relay, button, Pi link, wheel speed, hoverboard latch, CAN error counters, frame counts. It also has **E-stop** and **Release** buttons. |

**First DFU upload on Windows.** The STM32 bootloader (USB `0483:DF11`, "STM32 BOOTLOADER")
needs the WinUSB driver once. Put the board in DFU mode (hold BOOT0, tap NRST, release BOOT0),
run [Zadig](https://zadig.akeo.ie), select the device and install **WinUSB**. A board with no
firmware yet always has to be put in DFU mode by hand like this; after that the tool reboots it
over USB.

## Building and uploading without the tool

```bash
pio run                          # build (env: blackpill)
pio run -t upload                # upload over SWD with an ST-Link
pio run -e blackpill_dfu -t upload   # upload over USB DFU (board already in DFU mode)
pio device monitor               # debug console (any baud rate)
```

The same environments appear in the VS Code PlatformIO sidebar.

## What the firmware does

The behaviour follows the e-stop controller specification in
[RaspberryPi_ROS2/docs/hardware.md](../../RaspberryPi_ROS2/docs/hardware.md#e-stop-controller-stm32f411ceu6-body-node).
The frames are defined in
[hoverboard_can.dbc](../../Hoverboard_Firmware/docs/hoverboard_can.dbc).

**Trip.** The e-stop latches on any of these:
- `ESTOP = 1` (0x010) from the Pi.
- The e-stop button (debounced 20 ms).
- No `HB_CMD` (0x201) for **500 ms**, which means the Pi or the CAN bridge has died. This watchdog
  arms on the first `HB_CMD`.

A trip by the button or the watchdog is broadcast as `ESTOP = 1`, so the hoverboard brakes at once.
The first cause is kept.

**Brake, then cut power.** The relay stays closed while the hoverboard brakes; an unpowered board
would let the motors coast. It opens when either:
- both wheels are at or below 5 rpm for 150 ms, measured from the hoverboard's motion frame
  (0x281); or
- motion frames stop for 200 ms; or
- 1.5 s have passed, whatever the wheels are doing.

While the relay is still closed, `ESTOP = 1` is sent again whenever the hoverboard's status frame
(0x381) says it is not latched.

**Release.** Only on `ESTOP = 0` from the Pi, and **never while the button is pressed**. The relay
then closes and the hoverboard reboots. The watchdog re-arms on the next `HB_CMD`.

**Report.** `SAFETY_STATUS` (0x011) goes out at 10 Hz and immediately on any change. Byte 0
holds: bit 0 e-stop active, bit 1 relay closed, bit 2 button pressed, bits 4–6 the cause (0 none,
1 host request, 2 button, 3 host timeout). Byte 1 is a heartbeat counter. On the Pi,
`amr_hardware/can_bridge` publishes this on `/estop/status` and serves `/estop`.

**Lift.** `LIFT_CMD` (0x021) from the Pi asks for a height, a stop or a homing run. The first
"go to" after power-up homes first: each side runs down until its limit switch opens, and that is
height 0. Nothing moves before a command arrives. Each side is then driven to the target, with a
ramp, and trimmed against the other so the table stays level. The lift stops with a fault when a
switch is never reached, when a motor is driven for 500 ms without its encoder moving, or when the
sides drift too far apart. A home command clears the fault. While the e-stop is latched the PWM is
zero and the move is dropped. `LIFT_STATUS` (0x022) goes out at 10 Hz and on every state change.

`LIFT_SINGLE_CHANNEL` and `LIFT_LIMITS` in `config.h` select a single-channel driver (both motors
in parallel) and what the two switches are (one per side at the bottom, or bottom and top).

**Stack light and buzzer.** `BODY_CMD` (0x020) from the Pi chooses idle (green), moving (green with
blinking amber), paused (amber) or fault (red), and a buzzer pattern. With no frame for a second
the green light blinks: the controller is alive and waiting for the Pi. A latched e-stop overrides
all of it with a blinking red and three beeps.

**Robustness.**
- A 250 ms MCU watchdog: a hung firmware resets, and the relay output floats, so the pull-down
  opens the relay.
- The MCP2515 is checked every 100 ms and re-initialised if it stops answering, for example when
  unplugged.
- Bus-off and error-passive states are logged; bus-off recovery is automatic.
- Receive order is preserved across the MCP2515's two receive buffers, so an e-stop and its
  release are never swapped.
- Console output never blocks the safety loop. If nobody reads the USB port, output is dropped.

## Debug console

Open any serial terminal on the Blackpill's USB port; the baud rate does not matter. Type `help`.

| Command | Effect |
|---|---|
| `status` | E-stop and cause, relay, button, Pi link, hoverboard, CAN, uptime |
| `can` | Bitrate, bit timing and CNF registers, controller mode, TEC/REC, error flags |
| `stats` | Frames received/sent, transmit failures, overflows, bus-off events, console drops |
| `estop` / `release` | Latch, or release (also broadcasts `ESTOP = 0`; refused while the button is pressed) |
| `log rx` / `log all` / `log off` | Trace received, or all, CAN frames |
| `send ID B0 … B7` | Send a standard frame, in hex, for example `send 010 01` |
| `selftest` | MCP2515 loopback test (bus traffic paused for a few ms) |
| `lamptest` | All three lights on for 3 s and one beep |
| `lift` | Lift state, raw encoder counts, limit switches, duty |
| `lift home` / `lift stop` / `lift up` / `lift down` / `lift 25` | Home, stop, or go to a height in mm |
| `lift jog 200` | Open-loop nudge for 300 ms, positive = up. For bring-up: no limits are checked |
| `stream on` / `stream off` | Machine-readable `@S key=value` status lines every 200 ms (used by the tool) |
| `version`, `dfu`, `reboot` | Build info, reboot into the DFU bootloader, restart |

Events such as trips, releases, relay changes, the host link, CAN errors and watchdog resets are
printed with a timestamp as they happen.

## CAN bitrate and the MCP2515 crystal

**Is 500 kbit/s enough?** Yes. With the hoverboard at 50 Hz (command, odometry and motion frames),
its 10 Hz status, and this node's 10 Hz status, the bus carries about 20 kbit/s with worst-case bit
stuffing. The lift and light frames add little to that: about **5 % of 500 kbit/s**, half that at
1 Mbit/s.

The rule is simply that **every node uses the same bitrate**: the hoverboard
(`CAN_BITRATE` in `Hoverboard_Firmware/Inc/config.h`), the Pi
(`ip link set can0 … bitrate`, or `BitRate=` in systemd-networkd) and this board. The project
default is 1 Mbit/s.

| MCP2515 crystal | 1 Mbit/s | 500 kbit/s | 250 / 125 kbit/s |
|---|---|---|---|
| 8 MHz (most blue modules) | not possible | 8 TQ, sample point 62.5 % | 16 TQ, 68.75 % |
| 16 MHz | 8 TQ, 62.5 % | 16 TQ, 68.75 % | 16 TQ, 68.75 % |
| 20 MHz | 10 TQ, 70 % | 10 TQ, 70 % | 10 / 16 TQ |

- **8 MHz module:** select 500 kbit/s in the tool, and switch the hoverboard and the Pi to 500
  kbit/s as well.
- **The PCB:** fit a **16 MHz** crystal and keep 1 Mbit/s; nothing else changes.
- **Timing choice:** the synchronisation jump width is kept as large as the MCP2515 allows,
  because the hoverboard runs from its internal RC oscillator.

## Configuration

- [`include/config.h`](include/config.h): pins, hoverboard node ID, the safety timings above,
  `WATCHDOG_ARM_ON_BOOT`, `BOOT_LATCHED` (start with the relay open until the Pi releases), and
  the lift: driver type, switch arrangement, motor and encoder directions, travel, gains.
- [`platformio.ini`](platformio.ini): `CAN_BITRATE` and `MCP2515_CRYSTAL_HZ` (edited by the tool),
  and the USB product name "AMR Body Controller".

## Testing

The e-stop logic (`src/safety.cpp`), the lift controller (`src/lift.cpp`) and the light patterns
(`src/indicator.cpp`) have no hardware dependency, and each has a host test. The lift test runs
the controller against a simulated pair of lead screws: homing, moves, unequal motors, a jammed
side, a missing switch, the e-stop.

```bash
g++ -std=c++17 -Iinclude -Isrc test_host/safety_host_test.cpp src/safety.cpp -o safety_test && ./safety_test
g++ -std=c++17 -Iinclude -Isrc test_host/lift_host_test.cpp src/lift.cpp -o lift_test && ./lift_test
g++ -std=c++17 -Iinclude -Isrc test_host/indicator_host_test.cpp src/indicator.cpp -o indicator_test && ./indicator_test
```

On Windows without a compiler: `pip install ziglang`, then use `python -m ziglang c++` instead of
`g++`.

**On the robot:** follow the power-up order in [ELECTRICAL.md](../../ELECTRICAL.md#10-first-power-up-in-this-order),
the lift bring-up in its section 7, and then the first-drive checklist in
[hardware.md](../../RaspberryPi_ROS2/docs/hardware.md#6-first-drive-checklist).

## Project layout

```text
Blackpill_CAN_Software/
├── Blackpill Tool.bat          desktop tool launcher
├── platformio.ini              envs: blackpill (SWD), blackpill_dfu (USB DFU)
├── include/config.h            pins, timings, CAN and lift settings
├── include/protocol.h          CAN IDs and encodings (matches the DBC)
├── lib/MCP2515/                MCP2515 driver (bit timing, RX/TX, loopback)
├── src/main.cpp                CAN dispatch, relay, lights, LED, watchdog, status frames
├── src/safety.{h,cpp}          e-stop state machine (hardware independent)
├── src/lift.{h,cpp}            lift controller (hardware independent)
├── src/lift_hw.{h,cpp}         encoder timers, PWM, limit switches
├── src/indicator.{h,cpp}       stack light and buzzer patterns (hardware independent)
├── src/console.{h,cpp}         USB debug console
├── src/bootloader.{h,cpp}      reboot into the DFU bootloader
├── test_host/                  host tests: e-stop, lift, lights
└── tools/blackpill_gui.py      Blackpill Tool
```

## Status

| Verified | Not yet verified |
|---|---|
| Firmware builds without warnings (`-Wall -Wextra`), both environments | Anything on hardware |
| E-stop, lift and light logic: host tests pass | MCP2515 communication on the real module |
| Tool: Build, Find DFU device, Check ST-Link and the DFU wait / error path, run against the real PlatformIO, dfu-util and OpenOCD with no board connected | Relay, button, light and buzzer wiring |
| | Lift: timer and pin setup, motor and encoder directions, travel counts, gains |
| | The USB `dfu` reboot (uses an RTC backup register; if it doesn't work, use BOOT0 + NRST) |
| | Behaviour on the bus with the hoverboard and the Pi |
