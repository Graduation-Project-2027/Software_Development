// Host test of the stack light / buzzer logic (src/indicator.cpp), no hardware needed. From the project folder:
//   g++ -std=c++17 -Iinclude -Isrc test_host/indicator_host_test.cpp src/indicator.cpp -o indicator_test && ./indicator_test
// (any host compiler: g++, clang++, or "python -m ziglang c++" after "pip install ziglang")
#include <cstdio>

#include "indicator.h"
#include "protocol.h"

static int failures = 0;
#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::printf("  FAIL line %d: %s\n", __LINE__, #cond);         \
            failures++;                                                   \
        }                                                                 \
    } while (0)

static const Indicator::Config CFG = {1000, 3, 150};

struct Seen {
    int red = 0, amber = 0, green = 0, buzzer = 0, total = 0, beeps = 0;
};

// Runs update() every ms and counts how long each output was on, and the buzzer's rising edges.
static Seen run(Indicator &ind, uint32_t from, uint32_t to, bool estop, int hostMode = -1, int hostBuzz = 0) {
    Seen s;
    bool prevBuzz = false;
    for (uint32_t t = from; t < to; t++) {
        if (hostMode >= 0 && t % 200 == 0) ind.onHostFrame((uint8_t)hostMode, (uint8_t)hostBuzz, t);
        const Indicator::Outputs o = ind.update(t, estop);
        s.red += o.red;
        s.amber += o.amber;
        s.green += o.green;
        s.buzzer += o.buzzer;
        if (o.buzzer && !prevBuzz) s.beeps++;
        prevBuzz = o.buzzer;
        s.total++;
    }
    return s;
}

int main() {
    {
        std::puts("no host: green blinks slowly, nothing else");
        Indicator ind(CFG);
        Seen s = run(ind, 0, 4000, false);
        CHECK(s.green == 2000);
        CHECK(s.red == 0 && s.amber == 0 && s.buzzer == 0);
        CHECK(!ind.hostFresh(4000));
    }
    {
        std::puts("host idle: green steady");
        Indicator ind(CFG);
        Seen s = run(ind, 0, 2000, false, proto::IND_IDLE);
        CHECK(s.green == s.total);
        CHECK(s.red == 0 && s.amber == 0 && s.buzzer == 0);
    }
    {
        std::puts("host moving: green steady, amber blinks, slow travel beep");
        Indicator ind(CFG);
        Seen s = run(ind, 0, 4000, false, proto::IND_MOVING, proto::BUZZ_SLOW);
        CHECK(s.green == s.total);
        CHECK(s.amber == 2000);
        CHECK(s.red == 0);
        CHECK(s.beeps == 4 && s.buzzer == 400);
    }
    {
        std::puts("host paused: amber steady; host fault: red steady, fast beep");
        Indicator ind(CFG);
        Seen s = run(ind, 0, 1000, false, proto::IND_PAUSED);
        CHECK(s.amber == s.total && s.green == 0 && s.red == 0);
        s = run(ind, 1000, 2000, false, proto::IND_FAULT, proto::BUZZ_FAST);
        CHECK(s.red == s.total && s.green == 0 && s.amber == 0);
        CHECK(s.beeps == 5 && s.buzzer == 500);
    }
    {
        std::puts("host frames stop: falls back to the blinking green after the timeout");
        Indicator ind(CFG);
        run(ind, 0, 1000, false, proto::IND_FAULT);       // last frame at t = 800
        CHECK(ind.hostFresh(1800));
        CHECK(!ind.hostFresh(1801));
        Seen s = run(ind, 2000, 4000, false);
        CHECK(s.red == 0 && s.green == 1000);
    }
    {
        std::puts("e-stop: red blinks whatever the host asks, three beeps on the trip only");
        Indicator ind(CFG);
        run(ind, 0, 1000, false, proto::IND_MOVING, proto::BUZZ_SLOW);
        ind.onEstopTrip(1000);
        Seen s = run(ind, 1000, 5000, true, proto::IND_MOVING, proto::BUZZ_SLOW);
        CHECK(s.red == 2000);
        CHECK(s.green == 0 && s.amber == 0);
        CHECK(s.beeps == 3 && s.buzzer == 450);
    }
    {
        std::puts("unknown host values are treated as idle / buzzer off");
        Indicator ind(CFG);
        ind.onHostFrame(200, 200, 0);
        CHECK(ind.mode() == proto::IND_IDLE && ind.buzzer() == proto::BUZZ_OFF);
    }
    {
        std::puts("lamp test: everything on, then back to normal");
        Indicator ind(CFG);
        ind.lampTest(100, 3000);
        Seen s = run(ind, 100, 3100, false);
        CHECK(s.red == s.total && s.amber == s.total && s.green == s.total);
        CHECK(s.beeps == 1 && s.buzzer == 200);
        s = run(ind, 3100, 3200, false);
        CHECK(s.red == 0 && s.amber == 0);
    }
    {
        std::puts("millis() wrap-around: host frame stays fresh across the wrap");
        Indicator ind(CFG);
        ind.onHostFrame(proto::IND_PAUSED, 0, 0xFFFFFF00u);
        CHECK(ind.hostFresh(0x00000010u));
        CHECK(ind.update(0x00000010u, false).amber);
    }

    std::printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
    return failures ? 1 : 0;
}
