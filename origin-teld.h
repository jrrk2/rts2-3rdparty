/*
 * Celestron Origin Telescope Driver for RTS2
 *
 * DIAGNOSTIC / HYBRID HEADER
 * Compatible with legacy RTS2 virtual method layout
 */

#ifndef __RTS2_ORIGIN_TELD_H__
#define __RTS2_ORIGIN_TELD_H__

#include "teld.h"
#include <string>
#include <atomic>
#include <ctime>

// Forward declarations
class OriginWebSocket;
struct TelescopeStatus;

namespace rts2teld
{

class Origin : public Telescope
{
public:
    Origin(int argc, char **argv);
    virtual ~Origin();

protected:
    /* RTS2 lifecycle */
    virtual int processOption(int opt);
    virtual int initHardware();
    virtual int initValues();
    virtual int info();

    virtual bool needInfo();
    virtual bool isSafe();

    /* Motion / pointing */
    virtual int startResync();
    virtual int isMoving();
    virtual int stopMove();

    virtual int startPark();
    virtual int endPark();
    virtual int isParking();

    virtual int setTo(double set_ra, double set_dec);
    virtual int correct(double cor_ra, double cor_dec,
                        double real_ra, double real_dec);

    virtual void valueChanged(rts2core::Value *changed_value);

private:
    /* Connection */
    std::string telescopeHost;
    int telescopePort;
    bool useDiscovery;

    OriginWebSocket *webSocket;
    bool connected;

    bool ensureConnected();
    bool connectToTelescope();
    void disconnectFromTelescope();

    bool sendCommand(const std::string &command,
                     const std::string &destination,
                     const std::string &params = "");

    void pollMessages();
    void processMessage(const std::string &message);

    /* State */
    TelescopeStatus *status;
    int nextSequenceId;

    bool gotoInProgress;
    bool operationActive;
    bool debugEnabled;
    bool rawFrameLogging;

    /* Initialisation & site */
    bool initialising;
    bool siteLocationSet;
    bool have_valid_altaz;

    /* Timing */
    time_t lastRxTime;
    time_t lastTxTime;
    time_t lastStatusUpdate;
    time_t lastActivityTime;

    /* Status parsing */
    void updateTelescopeStatus(const std::string &jsonData);
    void updateConnectionPolicy();

    /* Coordinate conversion */
    void j2000ToJNow(double ra_j2000, double dec_j2000,
                     double *ra_jnow, double *dec_jnow);

    void jnowToJ2000(double ra_jnow, double dec_jnow,
                     double *ra_j2000, double *dec_j2000);

    /* Discovery */
    bool discoverTelescope();
    bool startDiscovery();
    void stopDiscovery();
    void pollDiscovery();

    int discoverySocket;
    bool discovering;
    time_t discoveryStartTime;

    /* RTS2 values */
    rts2core::ValueString *telescopeAddress;
    rts2core::ValueBool   *isAligned;
    rts2core::ValueDouble *batteryVoltage;
    rts2core::ValueDouble *temperature;
};

} // namespace rts2teld

#endif

