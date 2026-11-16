/*
 * AMR body controller configuration (STM32F411CEU6 Blackpill + MCP2515).
 *
 * Pins follow the WeAct Blackpill v2/v3 pinout (../Docs/Blackpill_Pinout.jpg). PA11/PA12 are USB,
 * PA13/PA14 SWD, PC13 the blue LED, PA0 the KEY button.
 */
#pragma once

#include <Arduino.h>

#define FW_NAME    "AMR body controller"
#define FW_VERSION "0.2.0"

// ---------------------------------------------------------------- CAN bus (MCP2515 on SPI1)
// Set from platformio.ini build_flags (the Blackpill Tool edits them). Every node on the bus must
// use the same bitrate: hoverboard CAN_BITRATE (Hoverboard_Firmware/Inc/config.h) and the Pi
// (ip link ... bitrate). An 8 MHz MCP2515 crystal reaches at most 500 kbit/s.
#ifndef CAN_BITRATE
#define CAN_BITRATE        1000000      // [bit/s] 125000, 250000, 500000 or 1000000
#endif
#ifndef MCP2515_CRYSTAL_HZ
#define MCP2515_CRYSTAL_HZ 16000000     // [Hz] crystal on the MCP2515 module: 8, 16 or 20 MHz
#endif

#define PIN_MCP_CS         PA4          // SPI1: SCK PA5, MISO PA6, MOSI PA7
#define PIN_MCP_INT        PB0          // MCP2515 INT (active low). Reserved: the firmware polls over SPI
#define MCP_SPI_HZ         8000000      // [Hz] MCP2515 maximum is 10 MHz

#define HOVERBOARD_NODE_ID 1            // CAN_NODE_ID of the hoverboard board (frames 0x200/0x280/0x380 + id)

// ---------------------------------------------------------------- I/O
#define PIN_RELAY          PB12         // relay that powers the hoverboard board
#define RELAY_ACTIVE_HIGH  1            // 1: HIGH closes the relay. Give the driver a pull-down so it stays open during reset
#define PIN_ESTOP_BUTTON   PB13         // e-stop button, normally closed to GND, internal pull-up
#define BUTTON_PRESSED_LEVEL HIGH       // NC contact: pressed (or a broken wire) reads HIGH
#define PIN_LED            PC13         // on-board blue LED
#define LED_ACTIVE_LOW     1

// Stack light and buzzer: 3.3 V logic outputs, each drives a low-side N-MOSFET (ELECTRICAL.md).
// Give every gate a pull-down so the outputs stay off while the MCU is in reset.
#define PIN_LIGHT_RED      PB14
#define PIN_LIGHT_AMBER    PB15
#define PIN_LIGHT_GREEN    PA8
#define PIN_BUZZER         PB5
#define OUTPUT_ACTIVE_HIGH 1            // 1: HIGH switches a light / the buzzer on

// ---------------------------------------------------------------- safety behaviour (RaspberryPi_ROS2/docs/hardware.md)
#define HOST_TIMEOUT_MS      500        // trip if no HB_CMD (0x200 + node) for this long (Pi / bridge crashed)
#define WATCHDOG_ARM_ON_BOOT 0          // 0: host watchdog arms on the first HB_CMD; 1: armed from power-up
#define BOOT_LATCHED         0          // 1: start with the e-stop latched (relay open) until the Pi releases it
#define BUTTON_DEBOUNCE_MS   20
#define BRAKE_STOP_RPM       5          // wheels count as stopped at or below this speed...
#define BRAKE_STOP_HOLD_MS   150        // ...for this long; then the relay opens
#define BRAKE_TIMEOUT_MS     1500       // open the relay at the latest after this long
#define MOTION_TIMEOUT_MS    200        // no motion frame for this long = board not reporting; open the relay
#define HB_STATUS_TIMEOUT_MS 300        // hoverboard status frame considered stale after this long
#define ESTOP_RESEND_MS      100        // re-send ESTOP=1 while the hoverboard still reports it is not latched
#define STATUS_PERIOD_MS     100        // SAFETY_STATUS (0x011) rate, plus immediately on change
#define CAN_RETRY_MS         1000       // retry MCP2515 initialisation after a failure

// ---------------------------------------------------------------- lift (src/lift.h, src/lift_hw.cpp)
// Two geared DC motors (RS775, 12 V) on a Cytron PWM + DIR driver, one quadrature encoder per motor and
// two limit switches. The timers are fixed by these pins: do not move them without changing lift_hw.cpp.
#define PIN_LIFT_PWM_L       PA2        // TIM9 CH1 -> Cytron PWM1
#define PIN_LIFT_PWM_R       PA3        // TIM9 CH2 -> Cytron PWM2
#define PIN_LIFT_DIR_L       PB8        // -> Cytron DIR1
#define PIN_LIFT_DIR_R       PB9        // -> Cytron DIR2
#define PIN_LIFT_ENC_L_A     PB6        // TIM4 CH1 (5 V tolerant)
#define PIN_LIFT_ENC_L_B     PB7        // TIM4 CH2
#define PIN_LIFT_ENC_R_A     PA15       // TIM2 CH1 (5 V tolerant)
#define PIN_LIFT_ENC_R_B     PB3        // TIM2 CH2
#define PIN_LIFT_LIMIT_A     PB10       // limit switch A, normally closed to GND, internal pull-up
#define PIN_LIFT_LIMIT_B     PB1        // limit switch B, normally closed to GND, internal pull-up
#define LIFT_LIMIT_ACTIVE_LEVEL HIGH    // NC contact: actuated (or a broken wire) reads HIGH

#define LIFT_SINGLE_CHANNEL  0          // 0: dual-channel driver, one channel per motor (MDD10A, MDD20A...)
                                        // 1: single-channel driver, both motors in parallel on PWM1 / DIR1
#define LIFT_LIMITS          Lift::LIMITS_PER_SIDE   // LIMITS_PER_SIDE: A = left bottom, B = right bottom
                                                     // LIMITS_BOTTOM_TOP: A = bottom, B = top
#define LIFT_INVERT_LEFT      0         // 1: this motor runs the wrong way (check with "lift jog 200": both must go UP)
#define LIFT_INVERT_RIGHT     0
#define LIFT_ENC_INVERT_LEFT  0         // 1: this encoder counts down while the lift goes up (check with "lift raw")
#define LIFT_ENC_INVERT_RIGHT 0

// MEASURE on the robot (README, "Lift bring-up"): home, jog to the top, read the counts with "lift raw".
#define LIFT_TRAVEL_MM        50        // [mm] table travel from the home switch to the top
#define LIFT_TRAVEL_COUNTS    20000     // [counts] encoder counts over that travel (4 x pulses per turn x gear ratio x turns)
#define LIFT_TOLERANCE_COUNTS 40        // "at target" window
#define LIFT_MAX_SKEW_COUNTS  800       // left/right difference that stops the lift with a fault
#define LIFT_PWM_HZ           20000     // Cytron drivers accept up to 20 kHz
#define LIFT_MAX_DUTY         800       // [0..1000] duty limit while moving
#define LIFT_MIN_DUTY         150       // smallest duty that still turns the screws under load
#define LIFT_HOMING_DUTY      300       // duty while homing (downwards)
#define LIFT_KP               400       // duty per 1000 counts of position error
#define LIFT_KSYNC            1500      // duty per 1000 counts of left/right difference
#define LIFT_RAMP_PER_MS      4         // duty change per ms (0 -> full in 250 ms)
#define LIFT_HOMING_TIMEOUT_MS 20000
#define LIFT_STALL_MS         500       // driven this long without the encoder moving: fault
#define LIFT_STATUS_PERIOD_MS 100       // LIFT_STATUS (0x022) rate, plus immediately on a state change

// ---------------------------------------------------------------- stack light and buzzer (src/indicator.h)
#define INDICATOR_TIMEOUT_MS 1000       // no BODY_CMD (0x020) for this long: green blinks ("waiting for the Pi")
#define ESTOP_BEEPS          3          // beeps when the e-stop trips
#define ESTOP_BEEP_MS        150        // length of each beep and of the pause after it

// ---------------------------------------------------------------- MCU watchdog
#define IWDG_TIMEOUT_US      250000     // a hung firmware resets the MCU; the relay output then floats (pull-down: open)

// ---------------------------------------------------------------- debug console (USB CDC, any baud rate)
#define STREAM_PERIOD_MS     200        // "@S" status lines for the Blackpill Tool after "stream on"
