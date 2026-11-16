#include "lift_hw.h"

#include <Arduino.h>

#include "config.h"
#include "console.h"

namespace liftHw {
namespace {

Lift lift_({
    LIFT_TRAVEL_COUNTS, LIFT_TOLERANCE_COUNTS, LIFT_MAX_SKEW_COUNTS, LIFT_MAX_DUTY, LIFT_MIN_DUTY, LIFT_HOMING_DUTY,
    LIFT_KP, LIFT_KSYNC, LIFT_RAMP_PER_MS, LIFT_HOMING_TIMEOUT_MS, LIFT_STALL_MS, LIFT_LIMITS, LIFT_SINGLE_CHANNEL != 0,
});

Raw raw_ = {0, 0, false, false, 0, 0};
uint16_t lastTim4_ = 0;
int32_t countsLeft_ = 0;
uint32_t pwmTop_ = 1000;
Lift::State lastState_ = Lift::UNHOMED;
int16_t jogDuty_ = 0;
uint32_t jogStart_ = 0, jogMs_ = 0;

void gpioAf(GPIO_TypeDef *port, uint32_t pin, uint32_t af, uint32_t pull) {
    GPIO_InitTypeDef g = {};
    g.Pin = pin;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = pull;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = af;
    HAL_GPIO_Init(port, &g);
}

// Encoder mode 3 (counts every edge of A and B), input filter of 8 timer clocks.
void encoderInit(TIM_TypeDef *tim, uint32_t top) {
    tim->CR1 = 0;
    tim->SMCR = 3;
    tim->CCMR1 = (1u << 0) | (3u << 4) | (1u << 8) | (3u << 12);
    tim->CCER = 0;
    tim->PSC = 0;
    tim->ARR = top;
    tim->CNT = 0;
    tim->EGR = TIM_EGR_UG;
    tim->CR1 = TIM_CR1_CEN;
}

void writeMotor(volatile uint32_t &ccr, uint32_t dirPin, int16_t duty, bool invert) {
    const bool up = (duty > 0) != invert;
    const uint32_t mag = (uint32_t)(duty < 0 ? -duty : duty);
    if (mag != 0) digitalWrite(dirPin, up ? HIGH : LOW);      // direction only changes at zero duty (lift.cpp ramps through 0)
    ccr = mag >= 1000 ? pwmTop_ : mag * pwmTop_ / 1000;
}

bool limitRead(uint32_t pin) { return digitalRead(pin) == LIFT_LIMIT_ACTIVE_LEVEL; }

int32_t toTenthMm(int32_t counts) { return (int32_t)((int64_t)counts * LIFT_TRAVEL_MM * 10 / LIFT_TRAVEL_COUNTS); }

const char *stateName(Lift::State s) {
    switch (s) {
        case Lift::UNHOMED: return "not homed";
        case Lift::HOMING:  return "homing";
        case Lift::IDLE:    return "holding";
        case Lift::MOVING:  return "moving";
        default:            return "FAULT";
    }
}

const char *faultName(Lift::Fault f) {
    switch (f) {
        case Lift::FAULT_HOMING_TIMEOUT: return "homing timeout (limit switch never reached)";
        case Lift::FAULT_STALL:          return "motor stalled or encoder not counting";
        case Lift::FAULT_SKEW:           return "left and right sides too far apart";
        default:                         return "none";
    }
}

}  // namespace

void begin() {
    // Outputs first, low, so the driver sees no PWM while the timers are set up.
    pinMode(PIN_LIFT_DIR_L, OUTPUT);
    pinMode(PIN_LIFT_DIR_R, OUTPUT);
    digitalWrite(PIN_LIFT_DIR_L, LOW);
    digitalWrite(PIN_LIFT_DIR_R, LOW);
    pinMode(PIN_LIFT_LIMIT_A, INPUT_PULLUP);
    pinMode(PIN_LIFT_LIMIT_B, INPUT_PULLUP);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM9_CLK_ENABLE();

    // PWM: TIM9 CH1 = PA2 (left), CH2 = PA3 (right), 20 kHz, PWM mode 1 with preload.
    uint32_t clk = HAL_RCC_GetPCLK2Freq();
    if (RCC->CFGR & RCC_CFGR_PPRE2_2) clk *= 2;       // APB prescaler > 1: timers run at twice the bus clock
    pwmTop_ = clk / LIFT_PWM_HZ;
    TIM9->CR1 = 0;
    TIM9->PSC = 0;
    TIM9->ARR = pwmTop_ - 1;
    TIM9->CCR1 = 0;
    TIM9->CCR2 = 0;
    TIM9->CCMR1 = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);
    TIM9->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM9->EGR = TIM_EGR_UG;
    TIM9->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    gpioAf(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, GPIO_AF3_TIM9, GPIO_PULLDOWN);

    // Encoders: left on TIM4 (PB6, PB7; 16 bit, extended in software), right on TIM2 (PA15, PB3; 32 bit).
    gpioAf(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_AF2_TIM4, GPIO_PULLUP);
    gpioAf(GPIOA, GPIO_PIN_15, GPIO_AF1_TIM2, GPIO_PULLUP);
    gpioAf(GPIOB, GPIO_PIN_3, GPIO_AF1_TIM2, GPIO_PULLUP);
    encoderInit(TIM4, 0xFFFF);
    encoderInit(TIM2, 0xFFFFFFFFu);
}

void update(uint32_t now, bool estop) {
    const uint16_t t4 = (uint16_t)TIM4->CNT;
    countsLeft_ += (int16_t)(t4 - lastTim4_);
    lastTim4_ = t4;
    raw_.countsLeft = LIFT_ENC_INVERT_LEFT ? -countsLeft_ : countsLeft_;
    const int32_t right = (int32_t)TIM2->CNT;
    raw_.countsRight = LIFT_ENC_INVERT_RIGHT ? -right : right;
    raw_.limitA = limitRead(PIN_LIFT_LIMIT_A);
    raw_.limitB = limitRead(PIN_LIFT_LIMIT_B);

    Lift::Inputs in = {raw_.countsLeft, raw_.countsRight, raw_.limitA, raw_.limitB, estop};
    Lift::Outputs out = lift_.update(now, in);

    if (jogMs_ != 0) {
        if (estop || now - jogStart_ >= jogMs_) jogMs_ = 0;
        else out = {jogDuty_, jogDuty_};
    }
    if (LIFT_SINGLE_CHANNEL) out.dutyRight = out.dutyLeft;

    raw_.dutyLeft = out.dutyLeft;
    raw_.dutyRight = out.dutyRight;
    writeMotor(TIM9->CCR1, PIN_LIFT_DIR_L, out.dutyLeft, LIFT_INVERT_LEFT != 0);
    writeMotor(TIM9->CCR2, PIN_LIFT_DIR_R, out.dutyRight, LIFT_INVERT_RIGHT != 0);

    const Lift::State s = lift_.state();
    if (s != lastState_) {
        char msg[96];
        if (s == Lift::FAULT) snprintf(msg, sizeof msg, "Lift FAULT: %s. Send a home command to clear it", faultName(lift_.fault()));
        else snprintf(msg, sizeof msg, "Lift %s at %d.%d mm", stateName(s), positionTenthMm() / 10, abs(positionTenthMm() % 10));
        console::event(msg);
        lastState_ = s;
    }
}

void goToTenthMm(uint16_t tenthMm) {
    jogMs_ = 0;
    lift_.goTo((int32_t)((int64_t)tenthMm * LIFT_TRAVEL_COUNTS / (LIFT_TRAVEL_MM * 10)));
}

void home() {
    jogMs_ = 0;
    lift_.home();
}

void stop() {
    jogMs_ = 0;
    lift_.stop();
}

void jog(int16_t duty, uint32_t ms, uint32_t now) {
    lift_.stop();
    jogDuty_ = duty > LIFT_MAX_DUTY ? LIFT_MAX_DUTY : duty < -LIFT_MAX_DUTY ? -LIFT_MAX_DUTY : duty;
    jogStart_ = now;
    jogMs_ = ms > 2000 ? 2000 : ms;
}

int16_t positionTenthMm() { return (int16_t)toTenthMm(lift_.homed() ? lift_.position() : 0); }
int16_t skewTenthMm() { return (int16_t)toTenthMm(lift_.homed() ? lift_.skew() : 0); }
const Raw &raw() { return raw_; }
Lift &controller() { return lift_; }

}  // namespace liftHw
