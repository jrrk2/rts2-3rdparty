#include <dome.h>
#include <logstream.h>

using namespace rts2dome;

class VirtualDome : public Dome
{
public:
    VirtualDome(int argc, char **argv)
        : Dome(argc, argv, DEVICE_TYPE_DOME, false),
          domestate(DS_CLOSED),
          ticks(0)
    {
        logStream(MESSAGE_INFO)
            << "RTS2 0.9.4 Virtual Dome started"
            << sendLog;
    }

protected:
    enum DomeState {
        DS_CLOSED,
        DS_OPEN,
        DS_MOVING_OPEN,
        DS_MOVING_CLOSE
    };

    DomeState domestate;
    int ticks;

    /* ---- centrald polls these ---- */

    long isOpened() override
    {
        return domestate == DS_OPEN;
    }

    long isClosed() override
    {
        return domestate == DS_CLOSED;
    }

    /* ---- commands from centrald ---- */

    int startOpen() override
    {
        logStream(MESSAGE_INFO) << "startOpen()" << sendLog;
        domestate = DS_MOVING_OPEN;
        ticks = 0;
        return 0;
    }

    int startClose() override
    {
        logStream(MESSAGE_INFO) << "startClose()" << sendLog;
        domestate = DS_MOVING_CLOSE;
        ticks = 0;
        return 0;
    }

    int endOpen() override
    {
        logStream(MESSAGE_INFO) << "endOpen()" << sendLog;
        return 0;
    }

    int endClose() override
    {
        logStream(MESSAGE_INFO) << "endClose()" << sendLog;
        return 0;
    }

    /* ---- called periodically ---- */

    int info() override
    {
        if (domestate == DS_MOVING_OPEN && ++ticks >= 2)
            domestate = DS_OPEN;

        if (domestate == DS_MOVING_CLOSE && ++ticks >= 2)
            domestate = DS_CLOSED;

        return 0;
    }

    int initHardware() override
    {
        domestate = DS_CLOSED;
        ticks = 0;
        return 0;
    }
};

int main(int argc, char **argv)
{
    VirtualDome dome(argc, argv);
    return dome.run();
}

