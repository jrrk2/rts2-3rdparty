#include <sensord.h>
#include <logstream.h>

using namespace rts2sensord;

class AlwaysGoodWeather : public SensorWeather
{
public:
    AlwaysGoodWeather(int argc, char **argv)
        // timeout = 0 → immediately good weather
        : SensorWeather(argc, argv, 0, "ALWAYSGOOD")
    {
        // 🔴 FORCE GOOD WEATHER IMMEDIATELY
        setWeatherState(true, "forced always-good weather (constructor)");

        logStream(MESSAGE_INFO)
            << "AlwaysGoodWeather sensor starting"
            << sendLog;
    }

    int initHardware() override
    {
    return 0;
    }

    int info() override
    {
    return 0;
    }

    int idle() override
    {
        return SensorWeather::idle();
    }
};

int main(int argc, char **argv)
{
    AlwaysGoodWeather sensor(argc, argv);
    return sensor.run();
}
