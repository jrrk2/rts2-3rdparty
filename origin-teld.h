/*
 * Celestron Origin Telescope Driver for RTS2
 * 
 * Direct Ethernet communication with Celestron Origin mount
 */

#ifndef __RTS2_ORIGIN_TELD_H__
#define __RTS2_ORIGIN_TELD_H__

#include "teld.h"
#include "altaz.h"
#include "connection/fork.h"
#include <string>
#include <memory>
#include <thread>
#include <atomic>

// Forward declarations for Qt-free implementations
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
        virtual int info();

        virtual bool needInfo();
        
        virtual int startResync();
        virtual int isMoving();
        virtual int stopMove();
        virtual int startPark();
        virtual int endPark();
        virtual int isParking();
        
        virtual int setTo(double set_ra, double set_dec);
        virtual int correct(double cor_ra, double cor_dec, double real_ra, double real_dec);
        
	virtual bool isSafe();
	virtual void valueChanged (rts2core::Value *changed_value) override;

    private:
        // Connection parameters
        std::string telescopeHost;
        int telescopePort;
        bool useDiscovery;  // NEW: Auto-discover telescope
        
        // WebSocket connection
        OriginWebSocket *webSocket;
        bool connected;
        
        // Telescope status
        TelescopeStatus *status;
        int nextSequenceId;
        
        // Coordinate tracking
        double targetRA;
        double targetDec;
        bool gotoInProgress;
        
        // Methods
        bool connectToTelescope();
        void disconnectFromTelescope();
        bool sendCommand(const std::string& command, const std::string& destination,
                        const std::string& params = "");
        void processMessage(const std::string& message);
        void updateTelescopeStatus(const std::string& jsonData);
        
        // Discovery
        bool discoverTelescope();
        bool startDiscovery();
        void stopDiscovery();
        void pollDiscovery();
        
        // Discovery state
        int discoverySocket;
        bool discovering;
        time_t discoveryStartTime;
        
        // Coordinate conversion
        void j2000ToJNow(double ra_j2000, double dec_j2000, double *ra_jnow, double *dec_jnow);
        void jnowToJ2000(double ra_jnow, double dec_jnow, double *ra_j2000, double *dec_j2000);
        
        // RTS2 values
        rts2core::ValueString *telescopeAddress;
        rts2core::ValueBool *isAligned;
        rts2core::ValueBool *trackingEnabled;
        rts2core::ValueDouble *batteryVoltage;
        rts2core::ValueDouble *temperature;
	std::thread keepaliveThread;
	std::atomic<bool> keepaliveRunning{false};
	std::thread rxThread;
	std::atomic<bool> rxRunning{false};
	bool siteLocationSet = false;
	bool raDecSet = false;
};

}

#endif // __RTS2_ORIGIN_TELD_H__
