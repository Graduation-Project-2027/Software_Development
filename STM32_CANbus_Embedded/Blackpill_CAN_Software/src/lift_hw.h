/*
 * Lift hardware: two quadrature encoders (TIM2, TIM4), two PWM + DIR outputs for the Cytron
 * driver (TIM9) and two limit switches. Runs the controller in lift.h. Pins: include/config.h.
 */
#pragma once

#include <stdint.h>

#include "lift.h"

namespace liftHw {

struct Raw {
    int32_t countsLeft, countsRight;
    bool limitA, limitB;
    int16_t dutyLeft, dutyRight;
};

void begin();
void update(uint32_t now, bool estop);

void goToTenthMm(uint16_t tenthMm);     // 0 = fully down
void home();
void stop();
// Open loop, both motors, for bring-up only: runs for `ms`, never while the e-stop is latched.
void jog(int16_t duty, uint32_t ms, uint32_t now);

int16_t positionTenthMm();              // mean of both sides
int16_t skewTenthMm();                  // left - right
const Raw &raw();
Lift &controller();

}  // namespace liftHw
