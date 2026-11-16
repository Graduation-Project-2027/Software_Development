/*
 * CAN protocol shared with the hoverboard firmware and the Pi (Hoverboard_Firmware/docs/hoverboard_can.dbc).
 * Standard 11-bit IDs, little-endian payloads.
 */
#pragma once

#include <stdint.h>

namespace proto {

constexpr uint16_t ID_ESTOP          = 0x010;  // Pi (or BODY) -> all: u8 bit0 1 = latch, 0 = release
constexpr uint16_t ID_SAFETY_STATUS  = 0x011;  // BODY -> Pi: see statusByte0()
constexpr uint16_t ID_BODY_CMD       = 0x020;  // Pi -> BODY: u8 light mode (Indication), u8 buzzer (Buzzer), 5 Hz
constexpr uint16_t ID_LIFT_CMD       = 0x021;  // Pi -> BODY: u8 command (LiftCommand), u16 target [0.1 mm]
constexpr uint16_t ID_LIFT_STATUS    = 0x022;  // BODY -> Pi: state, flags, position, skew, fault, heartbeat (10 Hz)
constexpr uint16_t ID_HB_ODOM_BASE   = 0x180;  // board -> Pi: i32 left/right hall ticks
constexpr uint16_t ID_HB_CMD_BASE    = 0x200;  // Pi -> board: i16 left rpm, i16 right rpm (50 Hz)
constexpr uint16_t ID_HB_MOTION_BASE = 0x280;  // board -> Pi: i16 rpm L/R, i16 current L/R [A x100]
constexpr uint16_t ID_HB_STATUS_BASE = 0x380;  // board -> Pi: battery, temperature, errors, flags, heartbeat

enum Cause : uint8_t {
    CAUSE_NONE         = 0,
    CAUSE_HOST_REQUEST = 1,
    CAUSE_BUTTON       = 2,
    CAUSE_HOST_TIMEOUT = 3,
};

// BODY_CMD byte 0: what the stack light shows while the e-stop is not latched.
enum Indication : uint8_t {
    IND_IDLE   = 0,   // green steady
    IND_MOVING = 1,   // green steady + amber blinking
    IND_PAUSED = 2,   // amber steady
    IND_FAULT  = 3,   // red steady
};

// BODY_CMD byte 1.
enum Buzzer : uint8_t {
    BUZZ_OFF  = 0,
    BUZZ_SLOW = 1,    // short beep every second (travel alarm)
    BUZZ_FAST = 2,    // 5 Hz beeping (lifting, warning)
};

// LIFT_CMD byte 0. Bytes 1-2: target height above the home switch, little-endian, 0.1 mm.
enum LiftCommand : uint8_t {
    LIFT_STOP = 0,    // hold where it is
    LIFT_GOTO = 1,    // go to the target (homes first after power-up)
    LIFT_HOME = 2,    // run down to the limit switches and zero there; clears a fault
};

// LIFT_STATUS byte 1.
constexpr uint8_t LIFT_FLAG_HOMED     = 0x01;
constexpr uint8_t LIFT_FLAG_AT_TARGET = 0x02;
constexpr uint8_t LIFT_FLAG_LIMIT_A   = 0x04;
constexpr uint8_t LIFT_FLAG_LIMIT_B   = 0x08;

inline const char *indicationName(uint8_t m) {
    switch (m) {
        case IND_IDLE:   return "idle";
        case IND_MOVING: return "moving";
        case IND_PAUSED: return "paused";
        case IND_FAULT:  return "fault";
        default:         return "?";
    }
}

inline const char *causeName(uint8_t c) {
    switch (c) {
        case CAUSE_NONE:         return "none";
        case CAUSE_HOST_REQUEST: return "host request";
        case CAUSE_BUTTON:       return "e-stop button";
        case CAUSE_HOST_TIMEOUT: return "host timeout";
        default:                 return "?";
    }
}

// SAFETY_STATUS byte 0: bit0 e-stop active, bit1 hoverboard power (relay closed), bit2 button pressed,
// bits 4-6 cause. Byte 1: heartbeat counter.
inline uint8_t statusByte0(bool active, bool power, bool button, uint8_t cause) {
    return (active ? 0x01 : 0) | (power ? 0x02 : 0) | (button ? 0x04 : 0) | ((cause & 0x07) << 4);
}

inline int16_t le16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

// Hoverboard status frame: status flags in byte 5, e-stop active = bit 3 (DBC signal bit 43).
constexpr uint8_t HB_STATUS_FLAGS_BYTE = 5;
constexpr uint8_t HB_STATUS_ESTOP_BIT  = 0x08;

}  // namespace proto
