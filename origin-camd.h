/*
 * Celestron Origin Camera Driver for RTS2
 * 
 * Direct Ethernet communication with Celestron Origin camera
 */

#ifndef __RTS2_ORIGIN_CAMD_H__
#define __RTS2_ORIGIN_CAMD_H__

#include "camd.h"
#include <string>
#include <memory>

// Forward declarations
class OriginWebSocket;
struct CameraStatus;

namespace rts2camd
{

class OriginCam : public Camera
{
    public:
        OriginCam(int argc, char **argv);
        virtual ~OriginCam();

    protected:
        virtual int processOption(int opt);
        virtual int initHardware();
        virtual int initChips();
        virtual int info();
        
        virtual int startExposure();
        virtual int stopExposure();
        virtual long isExposing();
        virtual int doReadout();
        
        virtual int setCoolTemp(float new_temp);
        virtual int switchCooling(bool cooling);
        
    private:
        // Connection parameters
        std::string telescopeHost;
        int telescopePort;
        bool useDiscovery;  // NEW: Auto-discover telescope
        
        // WebSocket connection
        OriginWebSocket *webSocket;
        bool connected;
        
        // Camera status
        CameraStatus *cameraStatus;
        int nextSequenceId;
        
        // Exposure tracking
        double exposureStart;
        double exposureDuration;
        bool exposureInProgress;
        bool imageReady;
        
        // Image data
        unsigned char *imageBuffer;
        size_t imageBufferSize;
        std::string pendingImagePath;
        double pendingImageRA;
        double pendingImageDec;
        
        // Methods
        bool connectToTelescope();
        void disconnectFromTelescope();
        bool sendCommand(const std::string& command, const std::string& destination,
                        const std::string& params = "");
        void processMessage(const std::string& message);
        void updateCameraStatus(const std::string& jsonData);
        void handleImageReady(const std::string& filePath, double ra, double dec);
        bool downloadAndProcessImage(const std::string& filePath);
        
        // Discovery
        bool discoverTelescope();
        bool startDiscovery();
        void stopDiscovery();
        void pollDiscovery();
        
        // Discovery state
        int discoverySocket;
        bool discovering;
        time_t discoveryStartTime;
        
        // RTS2 values
        rts2core::ValueString *telescopeAddress;
        rts2core::ValueInteger *gain;
        rts2core::ValueBool *previewMode;
        rts2core::ValueDouble *ccdTemp;
};

}

#endif // __RTS2_ORIGIN_CAMD_H__
