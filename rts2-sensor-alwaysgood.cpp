#include <sensord.h>
#include <logstream.h>
#include <value.h>

using namespace rts2sensord;
using namespace rts2core;

class AlwaysGoodWeather : public SensorWeather
{
public:
    AlwaysGoodWeather(int argc, char **argv)
        : SensorWeather(argc, argv, 0, "ALWAYSGOOD")
    {
        createValue(
            weather_open,
            "weather_open",
            "Weather allows opening",
            false,   // not writable
            0,       // flags
            0        // priority
        );

        weather_open->setValueBool(true);
        setWeatherState(true, "forced always-good weather");
        infoAll();

        logStream(MESSAGE_INFO)
            << "AlwaysGoodWeather sensor starting"
            << sendLog;
    }

    int idle() override
    {
        setWeatherState(true, "forced always-good weather");
        weather_open->setValueBool(true);
        infoAll();   // 🔑 updates infotime
        return SensorWeather::idle();
    }

private:
    ValueBool *weather_open;
};

int main(int argc, char **argv)
{
    AlwaysGoodWeather sensor(argc, argv);
    return sensor.run();
}

