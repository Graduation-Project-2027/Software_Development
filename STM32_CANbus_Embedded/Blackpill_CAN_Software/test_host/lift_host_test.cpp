// Host test of the lift controller (src/lift.cpp) against a simulated mechanism. From the project folder:
//   g++ -std=c++17 -Iinclude -Isrc test_host/lift_host_test.cpp src/lift.cpp -o lift_test && ./lift_test
// (any host compiler: g++, clang++, or "python -m ziglang c++" after "pip install ziglang")
#include <cstdio>

#include "lift.h"

static int failures = 0;
#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::printf("  FAIL line %d: %s\n", __LINE__, #cond);         \
            failures++;                                                   \
        }                                                                 \
    } while (0)

//                            travel tol skew  max  min home kp   ksync ramp homeT  stall limits                 single
static const Lift::Config CFG = {20000, 40, 800, 800, 150, 300, 400, 1500, 4, 20000, 500, Lift::LIMITS_PER_SIDE, false};

// Two lead screws. Speed is proportional to the duty; each side has its own gain, a start position
// above the bottom switch, and can be jammed. Raw counters start at an arbitrary value.
struct Plant {
    double left = 3000, right = 3400;        // counts above the bottom switch
    double gainL = 0.03, gainR = 0.03;       // counts per ms per duty unit
    bool jamL = false, jamR = false;
    bool topSwitch = false;                  // LIMITS_BOTTOM_TOP: limit B is the top switch
    double top = 20500;
    int32_t offL = 123456, offR = -7777;
    bool estop = false;
    int32_t maxSkew = 0;

    Lift::Inputs inputs() const {
        Lift::Inputs in;
        in.countsLeft = (int32_t)left + offL;
        in.countsRight = (int32_t)right + offR;
        in.limitA = left <= 0;
        in.limitB = topSwitch ? (left >= top || right >= top) : right <= 0;
        if (topSwitch) in.limitA = left <= 0 || right <= 0;
        in.estop = estop;
        return in;
    }
    void step(const Lift::Outputs &o) {
        if (!jamL) left += o.dutyLeft * gainL;
        if (!jamR) right += o.dutyRight * gainR;
        if (left < 0) left = 0;
        if (right < 0) right = 0;
    }
};

// Runs the loop every ms from `from` for at most `ms`; stops early when `until` state is reached.
static uint32_t run(Lift &lift, Plant &p, uint32_t from, uint32_t ms, int until = -1) {
    uint32_t t = from;
    for (; t < from + ms; t++) {
        const Lift::Outputs o = lift.update(t, p.inputs());
        p.step(o);
        if (lift.homed()) {
            const int32_t s = Lift::abs32(lift.skew());
            if (s > p.maxSkew) p.maxSkew = s;
        }
        if (until >= 0 && lift.state() == until && t > from + 5) break;
    }
    return t;
}

int main() {
    {
        std::puts("power-up: unhomed, no output until a command");
        Lift lift(CFG);
        Plant p;
        for (uint32_t t = 0; t < 1000; t++) {
            const Lift::Outputs o = lift.update(t, p.inputs());
            CHECK(o.dutyLeft == 0 && o.dutyRight == 0);
        }
        CHECK(lift.state() == Lift::UNHOMED && !lift.homed());
    }
    {
        std::puts("home: each side runs down to its own switch and becomes zero there");
        Lift lift(CFG);
        Plant p;
        lift.home();
        CHECK(lift.state() == Lift::HOMING);
        run(lift, p, 0, 15000, Lift::IDLE);
        CHECK(lift.state() == Lift::IDLE && lift.homed());
        CHECK(p.left == 0 && p.right == 0);
        CHECK(lift.positionLeft() == 0 && lift.positionRight() == 0);
        CHECK(lift.atTarget());
    }
    {
        std::puts("go to before homing: homes first, then reaches the target and stops");
        Lift lift(CFG);
        Plant p;
        lift.goTo(12000);
        CHECK(lift.state() == Lift::HOMING);
        uint32_t t = run(lift, p, 0, 15000, Lift::MOVING);
        CHECK(lift.homed());
        t = run(lift, p, t, 20000, Lift::IDLE);
        CHECK(lift.state() == Lift::IDLE && lift.atTarget());
        CHECK(Lift::abs32(lift.positionLeft() - 12000) <= CFG.toleranceCounts);
        CHECK(Lift::abs32(lift.positionRight() - 12000) <= CFG.toleranceCounts);
        const Lift::Outputs o = lift.update(t + 1, p.inputs());
        CHECK(o.dutyLeft == 0 && o.dutyRight == 0);

        std::puts("down again: back to 0, never below the switches");
        lift.goTo(0);
        run(lift, p, t + 2, 20000, Lift::IDLE);
        CHECK(lift.atTarget() && p.left >= 0 && p.right >= 0);
    }
    {
        std::puts("targets are clamped to the travel");
        Lift lift(CFG);
        lift.goTo(999999);
        CHECK(lift.target() == CFG.travelCounts);
        lift.goTo(-5);
        CHECK(lift.target() == 0);
    }
    {
        std::puts("unequal motors: the cross-coupling keeps the sides level");
        Lift lift(CFG);
        Plant p;
        p.gainR = 0.021;                      // right motor 30 % slower
        lift.goTo(18000);
        uint32_t t = run(lift, p, 0, 20000, Lift::MOVING);
        p.maxSkew = 0;
        run(lift, p, t, 40000, Lift::IDLE);
        CHECK(lift.state() == Lift::IDLE && lift.atTarget());
        CHECK(p.maxSkew < 400);
        std::printf("  largest left/right difference: %ld counts\n", (long)p.maxSkew);
    }
    {
        std::puts("jammed side: stall fault, both motors off, stays off until homed again");
        Lift lift(CFG);
        Plant p;
        lift.goTo(10000);
        uint32_t t = run(lift, p, 0, 15000, Lift::MOVING);
        t = run(lift, p, t, 300);
        p.jamR = true;
        t = run(lift, p, t, 5000, Lift::FAULT);
        CHECK(lift.state() == Lift::FAULT);
        CHECK(lift.fault() == Lift::FAULT_STALL || lift.fault() == Lift::FAULT_SKEW);
        const Lift::Outputs o = lift.update(t + 1, p.inputs());
        CHECK(o.dutyLeft == 0 && o.dutyRight == 0);
        lift.goTo(5000);
        CHECK(lift.state() == Lift::FAULT);
        p.jamR = false;
        lift.home();
        run(lift, p, t + 2, 20000, Lift::IDLE);
        CHECK(lift.state() == Lift::IDLE && lift.homed() && lift.fault() == Lift::FAULT_NONE);
    }
    {
        std::puts("switch never reached: homing times out");
        Lift lift(CFG);
        Plant p;
        p.left = p.right = 1e9;
        lift.home();
        run(lift, p, 0, CFG.homingTimeoutMs + 100, Lift::FAULT);
        CHECK(lift.state() == Lift::FAULT && lift.fault() == Lift::FAULT_HOMING_TIMEOUT);
    }
    {
        std::puts("e-stop: outputs off at once, move dropped, position kept, no restart on release");
        Lift lift(CFG);
        Plant p;
        lift.goTo(15000);
        uint32_t t = run(lift, p, 0, 15000, Lift::MOVING);
        t = run(lift, p, t, 400);
        p.estop = true;
        Lift::Outputs o = lift.update(t, p.inputs());
        CHECK(o.dutyLeft == 0 && o.dutyRight == 0);
        CHECK(lift.state() == Lift::IDLE && lift.homed());
        const int32_t held = lift.position();
        p.estop = false;
        t = run(lift, p, t + 1, 500);
        CHECK(lift.state() == Lift::IDLE && lift.position() == held);
        lift.goTo(15000);
        run(lift, p, t, 20000, Lift::IDLE);
        CHECK(lift.atTarget());
    }
    {
        std::puts("stop command: holds where it is");
        Lift lift(CFG);
        Plant p;
        lift.goTo(15000);
        uint32_t t = run(lift, p, 0, 15000, Lift::MOVING);
        t = run(lift, p, t, 400);
        lift.stop();
        const Lift::Outputs o = lift.update(t, p.inputs());
        CHECK(o.dutyLeft == 0 && o.dutyRight == 0 && lift.state() == Lift::IDLE && lift.atTarget());
    }
    {
        std::puts("bottom + top switches, one driver channel: homes on the bottom switch, top switch ends the travel");
        Lift::Config cfg = CFG;
        cfg.limits = Lift::LIMITS_BOTTOM_TOP;
        cfg.singleChannel = true;
        Lift lift(cfg);
        Plant p;
        p.topSwitch = true;
        p.top = 9000;                         // top switch below the configured travel
        p.left = p.right = 3000;
        lift.goTo(20000);
        uint32_t t = run(lift, p, 0, 15000, Lift::MOVING);
        CHECK(lift.homed());
        run(lift, p, t, 30000, Lift::IDLE);
        CHECK(lift.state() == Lift::IDLE);
        CHECK(p.left <= 9100 && p.right <= 9100);
        const Lift::Outputs o = lift.update(60000, p.inputs());
        CHECK(o.dutyLeft == 0 && o.dutyRight == 0);
    }

    std::printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
    return failures ? 1 : 0;
}
