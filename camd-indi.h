/*
 * INDI Camera Driver for RTS2
 * 
 * This driver allows RTS2 to control cameras through the INDI protocol.
 * It connects to an INDI server and manages camera operations including
 * exposure, temperature control, binning, and image download.
 *
 * Copyright (C) 2024
 * Licensed under GPL v2+
 */

#ifndef __RTS2_CAMD_INDI_H__
#define __RTS2_CAMD_INDI_H__

#include "camd.h"
#include <libindi/baseclient.h>
#include <libindi/basedevice.h>
#include <string>
#include <map>

namespace rts2camd
{

/**
 * INDI Camera Driver for RTS2
 * 
 * This class implements the RTS2 camera interface and acts as an INDI client
 * to control cameras through the INDI protocol.
 */
class IndiCam : public Camera, public INDI::BaseClient
{
    public:
        IndiCam(int argc, char **argv);
        virtual ~IndiCam();

    protected:
        // RTS2 Camera Interface Methods
        virtual int processOption(int opt);
        virtual int initHardware();
        virtual int initChips();
        virtual int info();
        virtual int startExposure();
        virtual int stopExposure();
        virtual int doReadout();
        virtual int setCoolTemp(float new_temp);
        virtual int switchCooling(bool cooling);
        
        // INDI Client Callback Methods
        virtual void newDevice(INDI::BaseDevice *dp);
        virtual void removeDevice(INDI::BaseDevice *dp);
        virtual void newProperty(INDI::Property property);
        virtual void removeProperty(INDI::Property property);
        virtual void newBLOB(IBLOB *bp);
        virtual void newSwitch(ISwitchVectorProperty *svp);
        virtual void newNumber(INumberVectorProperty *nvp);
        virtual void newText(ITextVectorProperty *tvp);
        virtual void newLight(ILightVectorProperty *lvp);
        virtual void newMessage(INDI::BaseDevice *dp, int messageID);
        virtual void serverConnected();
        virtual void serverDisconnected(int exit_code);

    private:
        // Connection parameters
        std::string indiServer;
        int indiPort;
        std::string cameraDevice;
        
        // INDI device and properties
        INDI::BaseDevice *camera;
        INumberVectorProperty *ccdExposure;
        INumberVectorProperty *ccdTemperature;
        INumberVectorProperty *ccdInfo;
        INumberVectorProperty *ccdBinning;
        INumberVectorProperty *ccdFrame;
        ISwitchVectorProperty *ccdConnection;
        ISwitchVectorProperty *ccdCooler;
        IBLOBVectorProperty *ccdImage;
        
        // State tracking
        bool deviceConnected;
        bool exposureInProgress;
        bool imageReady;
        float lastTemp;
        
        // Image buffer
        unsigned char *imageBuffer;
        size_t imageBufferSize;
        
        // Helper methods
        bool connectToIndiServer();
        bool connectCamera();
        bool disconnectCamera();
        bool waitForProperty(const char *device, const char *property, int timeout);
        void updateCameraInfo();
        void processImage(IBLOB *bp);
        bool sendNewNumber(INumberVectorProperty *nvp);
        bool sendNewSwitch(ISwitchVectorProperty *svp);
        INDI::BaseDevice *getDevice(const char *name);
        
        // Value storage
        rts2core::ValueString *indiServerAddress;
        rts2core::ValueInteger *indiServerPort;
        rts2core::ValueString *indiCameraName;
        rts2core::ValueBool *indiConnected;
};

}

#endif // __RTS2_CAMD_INDI_H__
