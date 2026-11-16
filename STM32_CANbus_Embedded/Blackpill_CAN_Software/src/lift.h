/*
 * Lift controller of the BODY node (no hardware access; time is passed in, in ms).
 *
 * Two geared DC motors (left / right lead screw), each with a quadrature encoder, on a Cytron
 * PWM + DIR driver, and two limit switches for homing.
 *
 * Homing : both sides run down slowly until their switch closes; that point is position 0.
 *          The first "go to" after power-up homes by itself. Nothing moves before a command.
 * Moving : each side is driven to the target by a proportional controller with a ramp; a
 *          cross-coupling term keeps the two sides level.
 * Faults : homing timeout, a motor that does not turn (stall, broken encoder), or the two
 *          sides drifting apart. A fault stops both motors until the next home command.
 * E-stop : outputs are 0 while it is latched and the move is dropped (the position is kept).
 *
 * Positions are encoder counts from the home switch, positive = up. Duties are -1000..1000,
 * positive = up.
 */
#pragma once

#include <stdint.h>

class Lift {
public:
    enum State : uint8_t { UNHOMED = 0, HOMING = 1, IDLE = 2, MOVING = 3, FAULT = 4 };
    enum Fault : uint8_t { FAULT_NONE = 0, FAULT_HOMING_TIMEOUT = 1, FAULT_STALL = 2, FAULT_SKEW = 3 };
    // What the two limit switches are.
    enum Limits : uint8_t {
        LIMITS_PER_SIDE = 0,     // A = left side at the bottom, B = right side at the bottom
        LIMITS_BOTTOM_TOP = 1,   // A = bottom (both sides), B = top (both sides)
    };

    struct Config {
        int32_t travelCounts;     // counts from home to the top of the travel
        int32_t toleranceCounts;  // "at target" window
        int32_t maxSkewCounts;    // left/right difference that raises FAULT_SKEW
        int16_t maxDuty;          // duty limit while moving [0..1000]
        int16_t minDuty;          // smallest duty that still moves the mechanism
        int16_t homingDuty;       // duty while homing (downwards)
        int16_t kp;               // duty per 1000 counts of position error
        int16_t kSync;            // duty per 1000 counts of left/right difference
        uint16_t rampPerMs;       // duty change per millisecond
        uint32_t homingTimeoutMs;
        uint32_t stallMs;         // driven for this long without the encoder moving: stall
        uint8_t limits;           // Limits
        bool singleChannel;       // both motors in parallel on one driver channel
    };
    struct Inputs {
        int32_t countsLeft, countsRight;  // raw encoder counts (any origin)
        bool limitA, limitB;              // true = switch actuated (or its wire broken)
        bool estop;
    };
    struct Outputs {
        int16_t dutyLeft, dutyRight;
    };

    explicit Lift(const Config &cfg) : cfg_(cfg) {}

    void home();                          // start (or restart) homing; also clears a fault
    void goTo(int32_t counts);            // homes first if needed
    void stop();                          // drop the move, hold here

    Outputs update(uint32_t now, const Inputs &in);

    State state() const { return state_; }
    Fault fault() const { return fault_; }
    bool homed() const { return homed_; }
    bool atTarget() const { return state_ == IDLE && homed_ && abs32(position() - target_) <= cfg_.toleranceCounts; }
    int32_t positionLeft() const { return posL_; }
    int32_t positionRight() const { return posR_; }
    int32_t position() const { return (posL_ + posR_) / 2; }
    int32_t skew() const { return posL_ - posR_; }
    int32_t target() const { return target_; }
    const Config &config() const { return cfg_; }

    static int32_t abs32(int32_t v) { return v < 0 ? -v : v; }

private:
    void setFault(Fault f);
    int16_t ramp(int16_t current, int16_t wanted, uint32_t dtMs) const;
    int16_t controller(int32_t error) const;
    bool stalled(uint32_t now, int16_t duty, int32_t counts, int32_t &lastCounts, uint32_t &movedAt) const;

    Config cfg_;
    State state_ = UNHOMED;
    Fault fault_ = FAULT_NONE;
    bool homed_ = false, pendingGoTo_ = false, started_ = false;
    bool homedL_ = false, homedR_ = false;
    int32_t zeroL_ = 0, zeroR_ = 0, posL_ = 0, posR_ = 0, target_ = 0;
    int16_t dutyL_ = 0, dutyR_ = 0;
    uint32_t last_ = 0, homingStart_ = 0;
    int32_t stallCountsL_ = 0, stallCountsR_ = 0;
    uint32_t movedAtL_ = 0, movedAtR_ = 0;
};
