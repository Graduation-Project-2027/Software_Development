#include "lift.h"

namespace {
int16_t clamp16(int32_t v, int32_t lo, int32_t hi) { return (int16_t)(v < lo ? lo : v > hi ? hi : v); }
}  // namespace

void Lift::setFault(Fault f) {
    fault_ = f;
    state_ = FAULT;
    pendingGoTo_ = false;
}

void Lift::home() {
    fault_ = FAULT_NONE;
    homed_ = homedL_ = homedR_ = false;
    state_ = HOMING;
    started_ = false;
}

void Lift::goTo(int32_t counts) {
    if (state_ == FAULT) return;                     // needs an explicit home first
    target_ = counts < 0 ? 0 : counts > cfg_.travelCounts ? cfg_.travelCounts : counts;
    if (!homed_) {
        pendingGoTo_ = true;
        if (state_ != HOMING) home();
        return;
    }
    if (abs32(position() - target_) > cfg_.toleranceCounts) {
        state_ = MOVING;
        started_ = false;
    }
}

void Lift::stop() {
    pendingGoTo_ = false;
    if (state_ == MOVING) {
        target_ = position();
        state_ = IDLE;
    } else if (state_ == HOMING) {
        state_ = UNHOMED;
    }
}

int16_t Lift::ramp(int16_t current, int16_t wanted, uint32_t dtMs) const {
    if (wanted == 0) return 0;                       // stopping is never delayed
    const int32_t step = (int32_t)cfg_.rampPerMs * (int32_t)(dtMs > 50 ? 50 : dtMs);
    // Reversing goes through zero first.
    if ((current > 0 && wanted < 0) || (current < 0 && wanted > 0)) return 0;
    if (wanted > current) return clamp16(current + step, -1000, wanted);
    return clamp16(current - step, wanted, 1000);
}

int16_t Lift::controller(int32_t error) const {
    if (abs32(error) <= cfg_.toleranceCounts) return 0;
    int32_t duty = error * cfg_.kp / 1000;
    if (duty > 0 && duty < cfg_.minDuty) duty = cfg_.minDuty;
    if (duty < 0 && duty > -cfg_.minDuty) duty = -cfg_.minDuty;
    return clamp16(duty, -cfg_.maxDuty, cfg_.maxDuty);
}

bool Lift::stalled(uint32_t now, int16_t duty, int32_t counts, int32_t &lastCounts, uint32_t &movedAt) const {
    if (duty == 0 || counts != lastCounts) {
        lastCounts = counts;
        movedAt = now;
        return false;
    }
    return now - movedAt >= cfg_.stallMs;
}

Lift::Outputs Lift::update(uint32_t now, const Inputs &in) {
    const uint32_t dt = now - last_;
    last_ = now;
    posL_ = in.countsLeft - zeroL_;
    posR_ = in.countsRight - zeroR_;

    const bool perSide = cfg_.limits == LIMITS_PER_SIDE;
    const bool bottomL = in.limitA;
    const bool bottomR = perSide ? in.limitB : in.limitA;
    const bool top = !perSide && in.limitB;

    int16_t wantL = 0, wantR = 0;

    if (in.estop) {
        // Outputs off at once. A move is dropped; homing has to be asked for again.
        if (state_ == MOVING) { target_ = position(); state_ = IDLE; }
        else if (state_ == HOMING) state_ = UNHOMED;
        pendingGoTo_ = false;
        dutyL_ = dutyR_ = 0;
        return {0, 0};
    }

    if (state_ == HOMING || state_ == MOVING) {
        if (!started_) {
            started_ = true;
            homingStart_ = now;
            movedAtL_ = movedAtR_ = now;
            stallCountsL_ = in.countsLeft;
            stallCountsR_ = in.countsRight;
        }
    }

    if (state_ == HOMING) {
        // A side that reaches its switch stops there and becomes that side's zero.
        if (bottomL && !homedL_) { homedL_ = true; zeroL_ = in.countsLeft; }
        if (bottomR && !homedR_) { homedR_ = true; zeroR_ = in.countsRight; }
        if (cfg_.singleChannel && (homedL_ || homedR_)) {
            // One channel cannot stop one side only: the first switch ends the homing.
            if (!homedL_) { homedL_ = true; zeroL_ = in.countsLeft; }
            if (!homedR_) { homedR_ = true; zeroR_ = in.countsRight; }
        }
        if (homedL_ && homedR_) {
            homed_ = true;
            posL_ = in.countsLeft - zeroL_;
            posR_ = in.countsRight - zeroR_;
            if (pendingGoTo_) {
                pendingGoTo_ = false;
                state_ = MOVING;
                started_ = false;
            } else {
                target_ = 0;
                state_ = IDLE;
            }
        } else if (now - homingStart_ >= cfg_.homingTimeoutMs) {
            setFault(FAULT_HOMING_TIMEOUT);
        } else {
            wantL = homedL_ ? 0 : -cfg_.homingDuty;
            wantR = homedR_ ? 0 : -cfg_.homingDuty;
        }
    } else if (state_ == MOVING) {
        if (abs32(skew()) > cfg_.maxSkewCounts) {
            setFault(FAULT_SKEW);
        } else {
            const int32_t errL = target_ - posL_, errR = target_ - posR_;
            if (cfg_.singleChannel) {
                wantL = wantR = controller(target_ - position());
            } else {
                // The side that is ahead is slowed down, the one behind is pushed.
                const int32_t sync = skew() * cfg_.kSync / 1000;
                wantL = controller(errL);
                wantR = controller(errR);
                if (wantL != 0) wantL = clamp16(wantL - (wantL > 0 ? sync : -sync), wantL > 0 ? cfg_.minDuty : -cfg_.maxDuty,
                                                wantL > 0 ? cfg_.maxDuty : -cfg_.minDuty);
                if (wantR != 0) wantR = clamp16(wantR + (wantR > 0 ? sync : -sync), wantR > 0 ? cfg_.minDuty : -cfg_.maxDuty,
                                                wantR > 0 ? cfg_.maxDuty : -cfg_.minDuty);
            }
            // The switches end the travel whatever the counters say.
            if (wantL < 0 && bottomL) wantL = 0;
            if (wantR < 0 && bottomR) wantR = 0;
            if (top && wantL > 0) wantL = 0;
            if (top && wantR > 0) wantR = 0;
            if (wantL == 0 && wantR == 0) state_ = IDLE;
        }
    }

    if (state_ != HOMING && state_ != MOVING) wantL = wantR = 0;

    dutyL_ = ramp(dutyL_, wantL, dt);
    dutyR_ = ramp(dutyR_, wantR, dt);

    if (state_ == HOMING || state_ == MOVING) {
        const bool sL = stalled(now, dutyL_, in.countsLeft, stallCountsL_, movedAtL_);
        const bool sR = stalled(now, dutyR_, in.countsRight, stallCountsR_, movedAtR_);
        if (sL || sR) {
            setFault(FAULT_STALL);
            dutyL_ = dutyR_ = 0;
        }
    }
    return {dutyL_, dutyR_};
}
