#include <dome.h>
#include <unistd.h>

using namespace rts2dome;

class VirtualDome : public Dome
{
public:
    VirtualDome(int argc, char **argv)
        : Dome(argc, argv),
          closing(false),
          opening(false),
          ticks(0)
    {}

    int initHardware() override
    {
        logStream(MESSAGE_INFO) << "VirtualDome init: CLOSED" << sendLog;
        closing = false;
        opening = false;
        ticks = 0;
        return 0;
    }

    int startClose() override
    {
        logStream(MESSAGE_INFO) << "startClose()" << sendLog;
        closing = true;
        opening = false;
        ticks = 0;
        return 0;
    }

    int startOpen() override
    {
        logStream(MESSAGE_INFO) << "startOpen()" << sendLog;
        opening = true;
        closing = false;
        ticks = 0;
        return 0;
    }

    long isClosed() override
    {
        if (!closing)
            return -2;

        if (++ticks > 3)
        {
            logStream(MESSAGE_INFO) << "VirtualDome: CLOSED" << sendLog;
            closing = false;
            return USEC_SEC;
        }
        return USEC_SEC;
    }

    long isOpened() override
    {
        if (!opening)
            return 0;

        if (++ticks > 3)
        {
            logStream(MESSAGE_INFO) << "VirtualDome: OPENED" << sendLog;
            opening = false;
            return 1;
        }
        return 0;
    }

    int endClose() override
    {
        logStream(MESSAGE_DEBUG) << "endClose() (called by centrald)" << sendLog;
        return 0;
    }

    int endOpen() override
    {
        logStream(MESSAGE_DEBUG) << "endOpen() (called by centrald)" << sendLog;
        return 0;
    }

private:
    bool closing;
    bool opening;
    int ticks;
};

int main(int argc, char **argv)
{
    VirtualDome dome(argc, argv);
    return dome.run();
}

