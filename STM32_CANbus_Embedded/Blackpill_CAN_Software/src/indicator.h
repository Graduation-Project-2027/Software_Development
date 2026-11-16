/*
 * Stack light (red / amber / green) and buzzer of the BODY node (no hardware access; time is passed in, in ms).
 *
 * The e-stop always wins and needs no host: red blinks and the buzzer gives three beeps on the trip.
 * Otherwise the Pi chooses the pattern with BODY_CMD (0x020); without a fresh frame the green light
 * blinks slowly ("controller alive, waiting for the Pi").
 *
 *   e-stop latched        red blinking 2 Hz                (+ 3 beeps when it trips)
 *   host: FAULT           red steady
 *   host: MOVING          green steady + amber blinking 1 Hz
 *   host: PAUSED          amber steady
 *   host: IDLE            green steady
 *   no host frame         green blinking 0.5 Hz
 */
#pragma once

#include <stdint.h>

class Indicator {
public:
    struct Config {
        uint32_t hostTimeoutMs;     // BODY_CMD older than this: fall back to "no host"
        uint32_t estopBeeps;        // beeps when the e-stop trips
        uint32_t estopBeepMs;       // length of each beep and of the pause after it
    };
    struct Outputs {
        bool red, amber, green, buzzer;
    };

    explicit Indicator(const Config &cfg) : cfg_(cfg) {}

    void onHostFrame(uint8_t mode, uint8_t buzzer, uint32_t now);
    void onEstopTrip(uint32_t now);
    void lampTest(uint32_t now, uint32_t durationMs);     // everything on (wiring check)

    Outputs update(uint32_t now, bool estopActive);

    bool hostFresh(uint32_t now) const { return hostSeen_ && now - lastHost_ <= cfg_.hostTimeoutMs; }
    uint8_t mode() const { return mode_; }
    uint8_t buzzer() const { return buzzer_; }

private:
    Config cfg_;
    uint8_t mode_ = 0, buzzer_ = 0;
    bool hostSeen_ = false;
    uint32_t lastHost_ = 0;
    bool tripBeeping_ = false;
    uint32_t tripAt_ = 0;
    bool testing_ = false;
    uint32_t testAt_ = 0, testMs_ = 0;
};
