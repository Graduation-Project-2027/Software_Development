#include "indicator.h"

#include "protocol.h"

void Indicator::onHostFrame(uint8_t mode, uint8_t buzzer, uint32_t now) {
    mode_ = mode <= proto::IND_FAULT ? mode : (uint8_t)proto::IND_IDLE;
    buzzer_ = buzzer <= proto::BUZZ_FAST ? buzzer : (uint8_t)proto::BUZZ_OFF;
    lastHost_ = now;
    hostSeen_ = true;
}

void Indicator::onEstopTrip(uint32_t now) {
    tripBeeping_ = true;
    tripAt_ = now;
}

void Indicator::lampTest(uint32_t now, uint32_t durationMs) {
    testing_ = true;
    testAt_ = now;
    testMs_ = durationMs;
}

Indicator::Outputs Indicator::update(uint32_t now, bool estopActive) {
    Outputs o = {false, false, false, false};

    if (testing_) {
        if (now - testAt_ < testMs_) {
            o.red = o.amber = o.green = true;
            o.buzzer = now - testAt_ < 200;
            return o;
        }
        testing_ = false;
    }

    if (tripBeeping_) {
        const uint32_t t = now - tripAt_;
        if (t >= 2 * cfg_.estopBeepMs * cfg_.estopBeeps) tripBeeping_ = false;
        else o.buzzer = (t / cfg_.estopBeepMs) % 2 == 0;
    }

    if (estopActive) {
        o.red = (now / 250) % 2 == 0;
        return o;
    }

    if (!hostFresh(now)) {
        o.green = (now / 1000) % 2 == 0;
        return o;
    }

    switch (mode_) {
        case proto::IND_FAULT:  o.red = true; break;
        case proto::IND_MOVING: o.green = true; o.amber = (now / 500) % 2 == 0; break;
        case proto::IND_PAUSED: o.amber = true; break;
        default:                o.green = true; break;
    }
    if (!tripBeeping_) {
        if (buzzer_ == proto::BUZZ_SLOW) o.buzzer = now % 1000 < 100;
        else if (buzzer_ == proto::BUZZ_FAST) o.buzzer = now % 200 < 100;
    }
    return o;
}
