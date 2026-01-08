/*
 * Celestron Origin Telescope Driver for RTS2 - Connect-on-Demand Version
 *
 * Direct Ethernet communication with Celestron Origin mount
 */

#ifndef __RTS2_ORIGIN_TELD_H__
#define __RTS2_ORIGIN_TELD_H__

#include "teld.h"
#include <string>

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
    virtual int processOption(int opt);
    virtual int initHardware();
    virtual int initValues();
    virtual int info();

    virtual bool needInfo();
    virtual bool isSafe();
    virtual void valueChanged(rts2core::Value *changed_value);

    virtual int startResync();
    virtual int isMoving();
    virtual int stopMove();
    virtual int startPark();
    virtual int endPark();
    virtual int isParking();

    virtual int setTo(double set_ra, double set_dec);
    virtual int correct(double cor_ra, double cor_dec, double real_ra, double real_dec);

private:
    // Connection parameters
    std::string telescopeHost;
    int telescopePort;
    bool useDiscovery;

    // WebSocket connection
    OriginWebSocket *webSocket;
    bool connected;

    // Operation state (NEW for connect-on-demand)
    bool operationActive;
    time_t lastStatusUpdate;

    // Telescope status
    TelescopeStatus *status;
    int nextSequenceId;

    // Coordinate tracking
    bool gotoInProgress;
    bool siteLocationSet;
    bool raDecSet;
    bool have_valid_altaz = false;

    // Discovery state
    int discoverySocket;
    bool discovering;
    time_t discoveryStartTime;

    // RTS2 values
    rts2core::ValueString *telescopeAddress;
    rts2core::ValueBool *isAligned;
    // tracking value is created by base Telescope class
    rts2core::ValueDouble *batteryVoltage;
    rts2core::ValueDouble *temperature;

    // Connection methods (NEW for connect-on-demand)
    bool ensureConnected();
    bool connectToTelescope();
    void disconnectFromTelescope();
    void pollMessages();

    // Communication
    bool sendCommand(const std::string& command,
                     const std::string& destination = "Mount",
                     const std::string& params = "");
    void processMessage(const std::string& message);
    void updateTelescopeStatus(const std::string& jsonData);

    // Discovery
    bool discoverTelescope();
    bool startDiscovery();
    void stopDiscovery();
    void pollDiscovery();

    // Coordinate conversion
    void j2000ToJNow(double ra_j2000, double dec_j2000, double *ra_jnow, double *dec_jnow);
    void jnowToJ2000(double ra_jnow, double dec_jnow, double *ra_j2000, double *dec_j2000);
};

}

#endif // __RTS2_ORIGIN_TELD_H__
