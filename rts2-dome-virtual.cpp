/*
 * RTS2 Virtual Dome Driver (RTS2 0.9.4)
 *
 * Minimal, correct dome simulator:
 *  - No geometry
 *  - No hardware
 *  - No block manipulation
 *  - Correct RTS2 state machine behaviour
 *
 * Tested logic against rts2/lib/rts2/dome.cpp
 */

#include "dome.h"

using namespace rts2dome;
using namespace rts2core;

class VirtualDome : public Dome
{
public:
    VirtualDome(int argc, char **argv)
        : Dome(argc, argv),
          state(CLOSED),
          transition_end(0)
    {
        logStream(MESSAGE_INFO) << "VirtualDome init: CLOSED" << sendLog;
    }

protected:
    enum DomeState {
        OPEN,
        CLOSED,
        OPENING,
        CLOSING
    };

    DomeState state;
    double transition_end;

    /* ---------- OPEN ---------- */

    virtual int startOpen() override
    {
        logStream(MESSAGE_INFO) << "VirtualDome: startOpen()" << sendLog;
        state = OPENING;
        transition_end = getNow() + 5;   // 5s simulated motion
        return 0;
    }

    virtual long isOpened() override
    {
        if (state == OPEN)
            return -2;

        if (state == OPENING) {
            if (getNow() >= transition_end) {
                state = OPEN;
                logStream(MESSAGE_INFO) << "VirtualDome: OPENED" << sendLog;
                return -2;
            }
            return USEC_SEC;
        }

        return -1;
    }

    virtual int endOpen() override
    {
        maskState(
            DOME_DOME_MASK | DEVICE_BLOCK_OPEN | DEVICE_BLOCK_CLOSE,
            DOME_OPENED | DEVICE_BLOCK_CLOSE,
            "virtual dome opened");

        return 0;
    }

    /* ---------- CLOSE ---------- */

    virtual int startClose() override
    {
        logStream(MESSAGE_INFO) << "VirtualDome: startClose()" << sendLog;
        state = CLOSING;
        transition_end = getNow() + 5;   // 5s simulated motion
        return 0;
    }

    virtual long isClosed() override
    {
        if (state == CLOSED)
            return -2;

        if (state == CLOSING) {
            if (getNow() >= transition_end) {
                state = CLOSED;
                logStream(MESSAGE_INFO) << "VirtualDome: CLOSED" << sendLog;
                return -2;
            }
            return USEC_SEC;
        }

        return -1;
    }

    virtual int endClose() override
    {
	maskState(
            DOME_DOME_MASK | DEVICE_BLOCK_OPEN | DEVICE_BLOCK_CLOSE,
            DOME_CLOSED | DEVICE_BLOCK_OPEN,
            "virtual dome closed");

        return 0;
    }
};

/* ---------- MAIN ---------- */

int main(int argc, char **argv)
{
    VirtualDome dome(argc, argv);
    return dome.run();
}

