# LEVERAGE AMR: electrical design and wiring

This is the wiring reference for the robot: power, the CAN bus, the e-stop chain, the lift, lights
and buzzer, and the pin-by-pin connections of the three controllers. The pin numbers here are the
ones compiled into the firmware. If you move a wire, change the matching `#define` and this file
together.

| Controller | Board | Firmware | Pins defined in |
|---|---|---|---|
| Robot computer | Raspberry Pi 5 (8 GB) | `RaspberryPi_ROS2` | `scripts/setup_pi_hardware.sh`, `amr_hardware/config/hardware.yaml` |
| Body controller (BODY) | WeAct Blackpill STM32F411CEU6 | `STM32_CANbus_Embedded/Blackpill_CAN_Software` | `include/config.h` |
| Drive | Hoverboard mainboard, STM32F103RCT6 | `Hoverboard_Firmware` (`VARIANT_CAN`) | `Inc/config.h`, `Inc/defines.h` |

Nothing in this document has been powered up yet. Section 10 lists what to check on the bench
before trusting it, and section 11 lists the decisions that are still yours.

## 1. Overview

```text
                 36 V battery (10S Li-ion, BMS)
                        |
                 main fuse, 58 V rated
                        |
                 main disconnect (XT90-S loop key)
                        |
       +----------------+--------------------+-------------------------+
       |                |                    |                         |
 hoverboard mainboard   36 V -> 5 V buck     36 V -> 12 V buck         charge port
 (motor bridges,        (logic rail)         (lift rail)
  always on the bus)        |                    |
       |                    |                    +--> Cytron driver --> 2 x RS775 lift motors
       |                    |
   HB PCB               PI BOX                                STM32 BOX
 +-------------+  RJ45  +--------------------+  RJ45  +--------------------------+
 | TJA1050     |<------>| Pi 5               |<------>| Blackpill F411           |
 | e-stop relay|  CAN   | MCP2515 + TJA1050  |  CAN   | MCP2515 + TJA1050        |
 | 120 ohm     |  5 V   | MPU9250 (I2C)      |  5 V   | 120 ohm                  |
 +-------------+  relay | 2 x RPLidar (USB)  |  relay | e-stop button            |
        |               | no termination     |        | lights, buzzer           |
   ON SWITCH            +--------------------+        | lift: PWM/DIR, encoders, |
   connector                                          | limit switches           |
                                                      +--------------------------+
```

The Pi box sits in the middle of the CAN bus. The HB PCB and the STM32 box are the two ends, so
those two carry the 120 Ω terminators and the Pi does not.

## 2. Power

### Battery and main feed

The pack is a 10S lithium-ion hoverboard battery: 42 V full, 36 V nominal, and the hoverboard
firmware stops driving at 33.7 V (`BAT_DEAD`).

- Put the main fuse within a few centimetres of the battery positive. Use a fuse rated for at least
  58 V DC (MIDI or ANL style). Ordinary automotive blade fuses are rated 32 V and can keep arcing
  at 42 V. 40 A suits the stock firmware limits (`I_DC_MAX` 17 A per motor).
- For the main disconnect I'd use an XT90-S anti-spark connector as a pull-out loop key. The
  mainboard's bulk capacitors charge with a hard spark on a plain switch, and the XT90-S has the
  precharge resistor built in.
- Do not pull the disconnect while the robot is moving. The motors regenerate into the bus when
  they brake, and with the battery gone the voltage has nowhere to go.

### Converters

| Rail | From | Feeds | Size |
|---|---|---|---|
| 5 V logic | 36 V | Pi 5, both lidars (through the Pi's USB), CAN modules, Blackpill, relay coil, 5 V lights | 5.1 V, 8 A or more |
| 12 V lift | 36 V | Cytron driver and the two RS775 motors. Optionally a 12 V stack light | see section 7 |

Pick converters with an input rating of 60 V or more. The LM2596 (40 V) and XL4005/XL4015 (about
36 V) modules from the ROV boards are not usable here: a full pack is already 42 V and braking
pushes it higher. The LM2596HV (57 V) works for small loads. Give each converter its own input
fuse.

The Pi 5 wants 5.1 V at up to 5 A. Feed it through header pins 2 and 4 (5 V) and 6 and 9 (GND),
with 20 AWG or thicker. Powered this way it cannot negotiate USB-PD, so it limits its USB ports to
600 mA in total, which is not enough for two lidar motors. `setup_pi_hardware.sh` writes
`usb_max_current_enable=1` to lift that limit.

### Ground

There is one ground: battery negative. The hoverboard's logic ground is battery negative too. Bring
the converter grounds and the battery negative together at one distribution point, with thick
wire, so that motor return current never looks for a path through the thin ground conductors of
the CAN cables.

### 5 V budget (estimates, measure yours)

| Load | Current |
|---|---|
| Pi 5 under navigation load | 2 to 3 A |
| 2 × RPLidar A1M8 | about 1 A, more while the motors spin up |
| 3 × CAN transceiver, MCP2515 ×2, Blackpill | 0.3 A |
| Relay coil (5 V, 70 Ω) | 0.07 A |
| Lights and buzzer on 5 V | up to 0.4 A |

## 3. Cables between the boxes

Both links are ordinary straight-through Cat5e patch cables (T568B at both ends), and both use the
same pinout, so the two cables are interchangeable.

| RJ45 pin | T568B colour | Signal | Notes |
|---|---|---|---|
| 1 | white/orange | CAN_H | twisted with pin 2 |
| 2 | orange | CAN_L | |
| 3 | white/green | ESTOP_RELAY | 5 V while the drive may be powered, 0 V = stop. Twisted with pin 6 |
| 6 | green | GND | relay return |
| 4 | blue | +5 V | both conductors in parallel |
| 5 | white/blue | +5 V | |
| 7 | white/brown | GND | both conductors in parallel |
| 8 | brown | GND | |

Why this layout:

- CAN gets its own twisted pair.
- The relay line shares a pair with ground. If that pair is crushed and shorts, the relay drops
  out, which is the safe direction.
- Each power pair carries a single potential. If someone plugs one of these cables into an
  Ethernet port, no DC flows through the port's transformers. Still, label the jacks and use a
  cable colour you don't use for networking. The Pi's own Ethernet jack is right there.
- The lift's 12 V does not go through these cables. An RJ45 contact is good for about 1.5 A.

In the Pi box the two jacks are wired pin to pin. The Pi's CAN transceiver taps pins 1 and 2 with a
short stub, the 5 V converter feeds pins 4 and 5 of each jack through a 0.75 A polyfuse, and pin 3
passes straight from one jack to the other without touching the Pi. Unplugging either cable
therefore releases the relay and stops the drive.

## 4. CAN bus

| Node | Controller | Transceiver | Terminator |
|---|---|---|---|
| HB PCB | bxCAN inside the STM32F103 | TJA1050 | 120 Ω fitted |
| Pi box | MCP2515 on SPI0 | TJA1050 | removed |
| STM32 box | MCP2515 on SPI1 | TJA1050 | 120 Ω fitted |

With the power off, a multimeter across CAN_H and CAN_L should read about 60 Ω.

Bitrate depends on the crystal on your MCP2515 boards. Read the marking on the can:

| Crystal | Bitrate | What to set |
|---|---|---|
| 16.000 MHz | 1 Mbit/s (project default) | nothing |
| 8.000 MHz | 500 kbit/s at most | `CAN_BITRATE 500000` in `Hoverboard_Firmware/Inc/config.h`; 500 kbit/s and 8 MHz in the Blackpill Tool; `--oscillator 8000000 --bitrate 500000` for the Pi script |

All three nodes must use the same bitrate. The Pi and the Blackpill each compute their timing from
their own crystal, so check both modules.

The hoverboard board has no crystal. It runs from an internal RC oscillator that can be up to
about 2 % off, so the other two nodes are set up to resynchronise generously (a wide SJW). The
Linux default is too tight for this, which is why `setup_pi_hardware.sh` sets the bit timing
explicitly instead of only a bitrate. If the `can_error_counter` in `/hoverboard/status` climbs
once the board is warm, drop every node to 500 kbit/s. The bus load is about 4 % at that speed.

Frames are listed in `Hoverboard_Firmware/docs/hoverboard_can.dbc`.

| ID | From → to | Content |
|---|---|---|
| 0x010 | Pi or BODY → all | e-stop: 1 latch, 0 release |
| 0x011 | BODY → Pi | e-stop state, relay, button, cause |
| 0x020 | Pi → BODY | stack light mode, buzzer |
| 0x021 | Pi → BODY | lift command and target height |
| 0x022 | BODY → Pi | lift state, height, left/right difference, fault |
| 0x201 | Pi → drive | wheel speeds, 50 Hz |
| 0x181, 0x281, 0x381 | drive → Pi | odometry ticks, speed and current, battery and status |

## 5. Raspberry Pi 5

| Header pin | GPIO | Connects to |
|---|---|---|
| 2, 4 | 5 V | 5 V converter output |
| 6, 9 | GND | converter ground |
| 1 | 3V3 | MPU9250 VCC |
| 3 | GPIO2 (SDA1) | MPU9250 SDA |
| 5 | GPIO3 (SCL1) | MPU9250 SCL |
| 14 | GND | MPU9250 GND |
| 19 | GPIO10 (MOSI) | MCP2515 SI |
| 21 | GPIO9 (MISO) | MCP2515 SO |
| 23 | GPIO11 (SCLK) | MCP2515 SCK |
| 24 | GPIO8 (CE0) | MCP2515 CS |
| 22 | GPIO25 | MCP2515 INT |
| 20 | GND | MCP2515 module GND |
| USB | | front lidar, rear lidar (CP2102 adapter boards) |

The Pi's GPIO is 3.3 V and not 5 V tolerant. SO and INT have to arrive at 3.3 V, the same
arrangement as on your ROV control PCB.

The MPU9250 sits flat with x forward, y left, z up. Keep it away from the hub motors and from the
5 V converter's inductor.

Run once on the Pi, then reboot:

```bash
sudo bash RaspberryPi_ROS2/scripts/setup_pi_hardware.sh          # 16 MHz crystal, 1 Mbit/s, INT on GPIO25
```

Give the lidars fixed names from `/dev/serial/by-path/` in `hardware.yaml`, otherwise front and
rear swap depending on which one enumerates first.

## 6. Body controller (Blackpill STM32F411CEU6)

Power the board through its 5V pin from RJ45 pins 4 and 5. The 3V3 pin is an output of the small
on-board regulator. It can run the encoders if they work at 3.3 V, and not much else.

| Pin | Direction | Signal | Connects to |
|---|---|---|---|
| PA4 | out | CAN CS | MCP2515 CS |
| PA5 | out | SPI1 SCK | MCP2515 SCK |
| PA6 | in | SPI1 MISO | MCP2515 SO |
| PA7 | out | SPI1 MOSI | MCP2515 SI |
| PB0 | in | CAN INT | MCP2515 INT. Reserved, the firmware polls |
| PB12 | out | ESTOP_RELAY drive | high-side switch to RJ45 pin 3. HIGH = drive powered |
| PB13 | in | e-stop button | NC contact to GND. Open = pressed |
| PB14 | out | red light | low-side MOSFET gate |
| PB15 | out | amber light | low-side MOSFET gate |
| PA8 | out | green light | low-side MOSFET gate |
| PB5 | out | buzzer | low-side MOSFET gate |
| PA2 | out | lift PWM, left | Cytron PWM1 (TIM9 CH1, 20 kHz) |
| PA3 | out | lift PWM, right | Cytron PWM2 (TIM9 CH2) |
| PB8 | out | lift direction, left | Cytron DIR1 |
| PB9 | out | lift direction, right | Cytron DIR2 |
| PB6 | in | left encoder A | TIM4 CH1 |
| PB7 | in | left encoder B | TIM4 CH2 |
| PA15 | in | right encoder A | TIM2 CH1 |
| PB3 | in | right encoder B | TIM2 CH2 |
| PB10 | in | limit switch A | NC contact to GND. Open = actuated |
| PB1 | in | limit switch B | NC contact to GND. Open = actuated |
| PC13 | out | status LED | on the board |
| PA11, PA12 | | USB | debug console, DFU uploads |
| PA13, PA14 | | SWD | ST-Link |

Free: PA1, PA9, PA10, PB4. I'd keep PB4 for a bumper strip if you add one. Leave PA0 (the KEY
button) and PB2 (BOOT1) alone.

### Relay drive (PB12)

PB12 cannot source a relay coil, and a 3.3 V pin cannot switch a 5 V high-side transistor off by
itself. Use two stages: PB12 through 1 kΩ to an NPN (or the PC817 you used on the ROV power PCB),
which pulls down the gate of a P-MOSFET whose source is on +5 V and whose drain goes to RJ45 pin 3.
Add 10 kΩ from PB12 to GND and 10 kΩ from the P-FET gate to +5 V.

Those two resistors are what make the circuit fail safe. While the Blackpill is in reset, being
flashed, or has been reset by its watchdog, PB12 floats. The pull-down keeps the NPN off, the
pull-up keeps the P-FET off, and the relay opens.

### Inputs

The e-stop button and both limit switches are wired normally closed to ground, so a cut wire or a
loose connector reads the same as "pressed". The internal pull-ups (about 40 kΩ) are enough on the
bench. On the robot, next to PWM wiring, add 4.7 kΩ to 3.3 V and 100 nF to ground at each of these
pins.

### Lights and buzzer

Each output drives a logic-level N-MOSFET (AO3400, IRLZ44N, or 2N7000 for a small LED): 100 Ω into
the gate, 10 kΩ from gate to ground, source to ground, drain to the load's negative lead. The
load's positive lead goes to a `V_LIGHT` terminal that you jumper to 5 V or to the 12 V lift rail,
depending on the light you buy. A common-anode tower light matches this directly. Put a diode
across a magnetic buzzer.

| What you see | Meaning |
|---|---|
| green blinking slowly | BODY alive, no Pi yet |
| green | idle |
| green with amber blinking | moving |
| amber | paused, or held by fleet traffic |
| red | fault (drive error, lift fault, failed task) |
| red blinking, three beeps | e-stop latched |

Type `lamptest` in the Blackpill console to switch everything on for three seconds.

## 7. Lift

Two RS775SH 12 V gearmotors, one per lead screw, each with a quadrature encoder. One Cytron driver
with PWM and DIR inputs. Two limit switches.

| Cytron | Connects to |
|---|---|
| PWM1, DIR1 | PA2, PB8 |
| PWM2, DIR2 | PA3, PB9 |
| GND (logic) | Blackpill GND |
| M1A/M1B | left motor |
| M2A/M2B | right motor |
| power input | 12 V lift rail, through its own fuse |

| Encoder wire | Connects to |
|---|---|
| left A, B | PB6, PB7 |
| right A, B | PA15, PB3 |
| VCC | 3.3 V if the encoder works there, otherwise 5 V (these four pins are 5 V tolerant) |
| GND | Blackpill GND |

Cytron's PWM and DIR inputs accept 3.3 V logic. Keep the motor leads twisted and away from the
encoder cables, and use shielded cable for the encoders if they run more than about 30 cm.

The firmware homes the lift by running each side down until its switch opens, and calls that
height zero. It then drives both sides to the commanded height and trims them against each other
so the table stays level. It stops with a fault if a switch is never reached, if a motor is
driven for half a second without its encoder moving, or if the two sides drift more than
`LIFT_MAX_SKEW_COUNTS` apart. While the e-stop is latched the PWM is zero, and the lift does not
resume by itself after a release.

Two things are configurable because I don't know which you have:

- `LIFT_SINGLE_CHANNEL`. 0 (default) is a dual-channel driver such as the MDD10A, one channel per
  motor. 1 is a single-channel driver with both motors in parallel on PWM1/DIR1. With one channel
  the firmware cannot level the two sides. It can only notice that they have drifted and stop.
- `LIFT_LIMITS`. `LIMITS_PER_SIDE` (default) means switch A is under the left side and switch B
  under the right, so each screw is zeroed separately. `LIMITS_BOTTOM_TOP` means A is the bottom
  and B is the top of the travel.

If you have the choice, a dual-channel driver with one bottom switch per side is the arrangement
that keeps two independent screws level.

### The 12 V lift rail

A stalled 775 motor can pull tens of amps, and there are two of them. Measure the running current
with a rack on the table before buying the 36 V to 12 V converter, and choose one with current
limiting. Until then, keep `LIFT_MAX_DUTY` low. Most Cytron drivers top out at 30 V, so the driver
cannot be fed from the battery directly.

### Bring-up

1. Lift unloaded, somewhere in mid travel. Open the console and type `lift`. Push each switch by
   hand and watch A and B change.
2. `lift jog 200`. Both sides must move up, briefly. If one goes down, set its `LIFT_INVERT_*`.
3. `lift` again. Both counts must have increased. If one decreased, set its `LIFT_ENC_INVERT_*`.
4. `lift home`. Both sides run down, stop on their switches, and the height reads 0.
5. Jog to the mechanical top, read the counts, and put slightly less into `LIFT_TRAVEL_COUNTS`.
   Measure the table rise and put it into `LIFT_TRAVEL_MM` (and `lift_height` in
   `mission_manager.yaml`, and `lift_range` in the URDF).
6. `lift up`, `lift down`, then try it with a rack and tune `LIFT_MAX_DUTY`, `LIFT_MIN_DUTY` and
   `LIFT_KP`.

## 8. Hoverboard mainboard and the HB PCB

| Mainboard point | Connects to |
|---|---|
| battery leads | battery, through the main fuse and disconnect |
| motor phases and hall connectors | hub motors, as stock. Robot's left wheel on the LEFT output |
| ON SWITCH connector (2 pins) | relay COM and NO on the HB PCB |
| PB8, MCU pin 61 | TJA1050 RXD |
| PB9, MCU pin 62 | TJA1050 TXD |
| a logic ground pad | HB PCB ground |
| SWD pads (3V3, SWCLK, GND, SWDIO) | ST-Link, for flashing |
| right sensor-board connector | USART3 debug at 115200. Its red wire is 15 V |

PB8 and PB9 are not routed anywhere on the board, so the two CAN wires are soldered to the MCU
pins themselves (61 and 62, next to the corner at pin 64). Use thin enamelled wire and glue it down
afterwards. `CAN_PINS 0` selects PA11/PA12 instead, which is no easier to reach.

The HB PCB holds a TJA1050 with its 120 Ω terminator and the e-stop relay:

| HB PCB part | Wiring |
|---|---|
| RJ45 pins 1, 2 | TJA1050 CANH, CANL |
| RJ45 pins 4, 5 | TJA1050 VCC (5 V) |
| RJ45 pins 6, 7, 8 | ground, joined to the mainboard's logic ground at one point |
| RJ45 pin 3 | relay coil +. Coil − to ground. Diode across the coil, cathode on pin 3 |
| relay COM, NO | the two pins of the ON SWITCH connector |

The TJA1050's 5 V RXD output can drive PB8 directly, because that pin is 5 V tolerant.

### Why the relay goes on the ON SWITCH connector

In the mainboard schematic (`Hoverboard_Firmware/docs/20150722_hoverboard_sch.pdf`, sheets 11 and
12) the battery is connected to the motor bridges permanently. The power button only enables the
small supply that feeds the gate drivers and the MCU. Closing the button's contact turns that
supply on, and without it the bridges cannot switch.

So the e-stop relay does not need to break the 40 A battery line. It replaces the push button and
switches about a milliamp. An SRD-05VDC or any small signal relay is fine. With the relay open the
MCU and gate drive are dead and the motors cannot be driven.

This needs firmware support, which is `POWER_SWITCH_EXTERNAL` in `Inc/config.h` (on by default for
`VARIANT_CAN`). With it the firmware never holds its own power latch, leaves the latch pin
floating, ignores the "button" (which now reads as held down for good) and never powers itself off
for inactivity. The stock firmware would sit at boot waiting for the button to be released.

To flash the board, put a jumper on the ON SWITCH connector so it stays powered.

## 9. Safety chain

What stops the robot, in the order things happen:

1. A trip. Any of: the e-stop button (PB13), an e-stop from mission control, or the Pi going
   silent for 500 ms.
2. BODY broadcasts the e-stop on CAN. The hoverboard brakes both wheels to zero under power, and
   keeps braking even if the Pi's commands have stopped. The lift PWM goes to zero.
3. When the wheels have stopped (or after 1.5 s, or 200 ms after the drive stops reporting), BODY
   drops PB12. The relay opens and the hoverboard's logic supply dies.
4. Release needs the button twisted out and a release from mission control. The relay closes and
   the hoverboard reboots.

And when something is broken rather than pressed:

| Failure | Result |
|---|---|
| Blackpill firmware hangs | watchdog resets it within 250 ms, PB12 floats, relay opens |
| Blackpill unpowered or being flashed | relay open |
| either RJ45 cable unplugged or cut | relay open |
| e-stop button wire broken | reads as pressed |
| Pi or the CAN bridge crashes | BODY trips after 500 ms |
| MCP2515 on BODY fails | BODY sees no commands, trips, opens the relay |
| hoverboard stops reporting | relay opens 200 ms into a stop |
| lift switch wire broken | reads as actuated. The lift will not drive past it |

Limits you should know about:

- The button goes through software. It is a brake-then-cut stop, and it depends on the Blackpill
  running (backed by the watchdog and the pull-down). If your examiners want a hardwired stop, use
  a button with two NC contacts and put the second one in series with RJ45 pin 3 inside the STM32
  box. The relay then drops the instant the button is pressed, whatever the firmware is doing. The
  price is that the wheels coast instead of braking, because the hoverboard loses power before it
  can brake.
- There is no holding brake. With the drive unpowered the wheels turn freely, and on a slope the
  robot rolls.
- The lidars are not safety-rated sensors. Obstacle avoidance is a navigation feature.
- The main disconnect is the only thing that removes battery voltage from the motor bridges. Pull
  it before working on the wiring.

## 10. First power-up, in this order

1. Battery disconnected. Check 60 Ω across CAN_H/CAN_L, no short from +5 V to ground on either
   RJ45, and continuity of pin 3 from the STM32 box jack to the HB PCB relay.
2. Flash the hoverboard with a jumper on its ON SWITCH connector. Remove the jumper: the board
   must switch off within about a second. Refit it: the board must boot and beep. This is the test
   of the latch wiring in section 8. If the board stays on with the jumper removed, stop and tell
   me which board revision you have.
3. Flash the Blackpill over USB. With no CAN yet its LED flickers fast. Type `status`, `selftest`
   and `lamptest`.
4. Power the 5 V rail with the motors' battery still disconnected. `candump can0` on the Pi must
   show 0x011 and 0x022 from BODY.
5. Press and release the e-stop button and watch the relay and `status`.
6. Wheels off the ground. Connect the battery. Then follow the first-drive checklist in
   `RaspberryPi_ROS2/docs/hardware.md`.
7. Lift bring-up from section 7, last, and unloaded first.

## 11. Still open

- Crystal frequency on your two MCP2515 boards (section 4).
- Which Cytron driver, and how the two limit switches are placed (section 7).
- Lift motor current under load, which sizes the 12 V converter.
- `LIFT_TRAVEL_COUNTS`, `LIFT_TRAVEL_MM`, wheel radius and wheel separation are placeholders until
  measured.
- Battery calibration (`BAT_CALIB_REAL_VOLTAGE`, `BAT_CALIB_ADC`) in the hoverboard `config.h` is
  still the upstream example. The battery percentage is wrong until you calibrate it.
- Whether the latch circuit on your mainboard matches the 2015 schematic. Step 2 above answers it.
