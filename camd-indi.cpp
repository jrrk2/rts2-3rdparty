/*
 * INDI Camera Driver for RTS2 - Implementation
 * 
 * This driver allows RTS2 to control cameras through the INDI protocol.
 */

#include "camd-indi.h"
#include <unistd.h>
#include <cstring>
#include <fitsio.h>
#include <cmath>

using namespace rts2camd;

static INDI::BaseDevice *asDevicePtr(INDI::BaseDevice *d) { return d; }
static INDI::BaseDevice *asDevicePtr(INDI::BaseDevice &d) { return &d; }

IndiCam::IndiCam(int argc, char **argv)
    : Camera(argc, argv), INDI::BaseClient()
{
    indiServer = "localhost";
    indiPort = 7624;
    cameraDevice = "";
    
    camera = nullptr;
    ccdExposure = nullptr;
    ccdTemperature = nullptr;
    ccdInfo = nullptr;
    ccdBinning = nullptr;
    ccdFrame = nullptr;
    ccdConnection = nullptr;
    ccdCooler = nullptr;
    ccdImage = nullptr;
    
    deviceConnected = false;
    exposureInProgress = false;
    imageReady = false;
    lastTemp = -273.15;
    
    imageBuffer = nullptr;
    imageBufferSize = 0;
    
    // Add RTS2 configuration values
    createValue(indiServerAddress, "INDI_SERVER", "INDI server hostname or IP", false);
    indiServerAddress->setValueCharArr(indiServer.c_str());
    
    createValue(indiServerPort, "INDI_PORT", "INDI server port", false);
    indiServerPort->setValueInteger(indiPort);
    
    createValue(indiCameraName, "INDI_CAMERA", "INDI camera device name", false);
    indiCameraName->setValueCharArr(cameraDevice.c_str());
    
    createValue(indiConnected, "INDI_CONNECTED", "INDI connection status", false);
    indiConnected->setValueBool(false);
    
    // Add command line options
    addOption('s', "indi-server", 1, "INDI server hostname (default: localhost)");
    addOption('p', "indi-port", 1, "INDI server port (default: 7624)");
    addOption('c', "indi-camera", 1, "INDI camera device name (required)");
}

IndiCam::~IndiCam()
{
    if (imageBuffer)
    {
        delete[] imageBuffer;
        imageBuffer = nullptr;
    }
    
    if (deviceConnected)
    {
        disconnectCamera();
    }
    
    disconnectServer();
}

int IndiCam::processOption(int opt)
{
    switch (opt)
    {
        case 's':
            indiServer = optarg;
            indiServerAddress->setValueCharArr(indiServer.c_str());
            return 0;
            
        case 'p':
            indiPort = atoi(optarg);
            indiServerPort->setValueInteger(indiPort);
            return 0;
            
        case 'c':
            cameraDevice = optarg;
            indiCameraName->setValueCharArr(cameraDevice.c_str());
            return 0;
            
        default:
            return Camera::processOption(opt);
    }
}

int IndiCam::initHardware()
{
    if (cameraDevice.empty())
    {
        logStream(MESSAGE_ERROR) << "INDI camera device name not specified. Use -c option." << sendLog;
        return -1;
    }
    
    if (!connectToIndiServer())
    {
        logStream(MESSAGE_ERROR) << "Failed to connect to INDI server at " 
                                << indiServer << ":" << indiPort << sendLog;
        return -1;
    }
    
    // Then use this block where you currently scan getDevices()
for (auto &item : getDevices())
{
    INDI::BaseDevice *d = asDevicePtr(item);
    if (!d) continue;

    const char *dn = d->getDeviceName();
    if (!dn) continue;

    std::string devName(dn);

    logStream(MESSAGE_INFO)
        << "Found INDI device: '" << devName << "'" << sendLog;

    // exact match (or switch to find() if you want fuzzy)
    if (devName == cameraDevice)
    {
        camera = d;
        logStream(MESSAGE_INFO)
            << "Matched camera device: " << devName << sendLog;
        break;
    }
}

    // Wait for the camera device to appear
    logStream(MESSAGE_INFO) << "Waiting for camera device: " << cameraDevice << sendLog;
    
    int timeout = 30; // 30 seconds
    while (timeout > 0 && !camera)
    {
        sleep(1);
        timeout--;
    }
    
    if (!camera)
    {
        logStream(MESSAGE_ERROR) << "Camera device " << cameraDevice << " not found" << sendLog;
        return -1;
    }
    
    // Connect to the camera
    if (!connectCamera())
    {
        logStream(MESSAGE_ERROR) << "Failed to connect to camera device" << sendLog;
        return -1;
    }
    
    // Wait for essential properties
    if (!waitForProperty(cameraDevice.c_str(), "CCD_EXPOSURE", 10) ||
        !waitForProperty(cameraDevice.c_str(), "CCD_INFO", 10))
    {
        logStream(MESSAGE_ERROR) << "Essential camera properties not available" << sendLog;
        return -1;
    }
    
    updateCameraInfo();
    
    logStream(MESSAGE_INFO) << "INDI camera initialized successfully" << sendLog;
    indiConnected->setValueBool(true);
    
    return 0;
}

int IndiCam::initChips()
{
    if (!ccdInfo)
    {
        logStream(MESSAGE_ERROR) << "CCD_INFO property not available" << sendLog;
        return -1;
    }

    INumber *w = IUFindNumber(ccdInfo, "CCD_MAX_X");
    INumber *h = IUFindNumber(ccdInfo, "CCD_MAX_Y");
    INumber *px = IUFindNumber(ccdInfo, "CCD_PIXEL_SIZE");
    INumber *py = IUFindNumber(ccdInfo, "CCD_PIXEL_SIZE_Y");

    if (!w || !h)
    {
        logStream(MESSAGE_ERROR) << "Unable to get CCD dimensions" << sendLog;
        return -1;
    }

    const int width  = static_cast<int>(w->value);
    const int height = static_cast<int>(h->value);

    const double pixelX = px ? px->value : 1.0;
    const double pixelY = py ? py->value : pixelX;

    /* This is the ONLY correct call */
    initCameraChip(width, height, pixelX, pixelY);

    logStream(MESSAGE_INFO)
        << "Chip initialized: "
        << width << "x" << height
        << " px=" << pixelX << "µm"
        << sendLog;

    return 0;
}

int IndiCam::info()
{
    if (ccdTemperature)
    {
        INumber *temp = IUFindNumber(ccdTemperature, "CCD_TEMPERATURE_VALUE");
        if (temp)
        {
            lastTemp = temp->value;
            tempSet->setValueDouble(lastTemp);
        }
    }
    
    return Camera::info();
}

int IndiCam::startExposure()
{
    if (!ccdExposure)
    {
        logStream(MESSAGE_ERROR) << "CCD_EXPOSURE property not available" << sendLog;
        return -1;
    }
    
    // Set frame parameters if available
    if (ccdFrame)
    {
        INumber *x = IUFindNumber(ccdFrame, "X");
        INumber *y = IUFindNumber(ccdFrame, "Y");
        INumber *w = IUFindNumber(ccdFrame, "WIDTH");
        INumber *h = IUFindNumber(ccdFrame, "HEIGHT");
        
        if (x && y && w && h)
        {
            x->value = chipTopX();
            y->value = chipTopY();
            w->value = getUsedWidth();
            h->value = getUsedHeight();
            
            sendNewNumber(ccdFrame);
        }
    }
    
    // Set binning if available
    if (ccdBinning)
    {
        INumber *hbin = IUFindNumber(ccdBinning, "HOR_BIN");
        INumber *vbin = IUFindNumber(ccdBinning, "VER_BIN");
        
        if (hbin && vbin)
        {
            hbin->value = binningHorizontal();
            vbin->value = binningVertical();
            
            sendNewNumber(ccdBinning);
        }
    }
    
    // Start exposure
    INumber *exp = IUFindNumber(ccdExposure, "CCD_EXPOSURE_VALUE");
    if (!exp)
    {
        logStream(MESSAGE_ERROR) << "CCD_EXPOSURE_VALUE not found" << sendLog;
        return -1;
    }
    
    exp->value = getExposure();
    exposureInProgress = true;
    imageReady = false;
    
    if (!sendNewNumber(ccdExposure))
    {
        logStream(MESSAGE_ERROR) << "Failed to start exposure" << sendLog;
        exposureInProgress = false;
        return -1;
    }
    
    logStream(MESSAGE_INFO) << "Exposure started: " << getExposure() << " seconds" << sendLog;
    
    return 0;
}

int IndiCam::stopExposure()
{
    if (!exposureInProgress)
        return 0;
    
    if (ccdExposure)
    {
        // INDI doesn't have a standard abort, but setting exposure to 0 should stop it
        INumber *exp = IUFindNumber(ccdExposure, "CCD_EXPOSURE_VALUE");
        if (exp)
        {
            exp->value = 0;
            sendNewNumber(ccdExposure);
        }
    }
    
    exposureInProgress = false;
    imageReady = false;
    
    logStream(MESSAGE_INFO) << "Exposure stopped" << sendLog;
    
    return 0;
}

int IndiCam::doReadout()
{
    if (!imageReady)
    {
        logStream(MESSAGE_ERROR) << "No image ready for readout" << sendLog;
        return -1;
    }
    
    if (!imageBuffer || imageBufferSize == 0)
    {
        logStream(MESSAGE_ERROR) << "No image data available" << sendLog;
        return -1;
    }
    
    // Copy image data to RTS2 buffer (chip 0)
    int dataSize = getUsedWidth() * getUsedHeight() * sizeof(uint16_t);
    
    if ((size_t)dataSize > imageBufferSize)
    {
        logStream(MESSAGE_ERROR) << "Image size mismatch" << sendLog;
        return -1;
    }
    
    memcpy(getDataBuffer(0), imageBuffer, dataSize);
    
    imageReady = false;
    
    logStream(MESSAGE_INFO) << "Image readout complete" << sendLog;
    
    return 0;
}

int IndiCam::setCoolTemp(float new_temp)
{
    if (!ccdTemperature)
    {
        logStream(MESSAGE_WARNING) << "Temperature control not available" << sendLog;
        return -1;
    }
    
    INumber *temp = IUFindNumber(ccdTemperature, "CCD_TEMPERATURE_VALUE");
    if (!temp)
    {
        logStream(MESSAGE_ERROR) << "CCD_TEMPERATURE_VALUE not found" << sendLog;
        return -1;
    }
    
    temp->value = new_temp;
    
    if (!sendNewNumber(ccdTemperature))
    {
        logStream(MESSAGE_ERROR) << "Failed to set temperature" << sendLog;
        return -1;
    }
    
    logStream(MESSAGE_INFO) << "Temperature set to " << new_temp << "°C" << sendLog;
    
    return 0;
}

int IndiCam::switchCooling(bool cooling)
{
    if (!ccdCooler)
    {
        logStream(MESSAGE_WARNING) << "Cooler control not available" << sendLog;
        return -1;
    }
    
    ISwitch *coolerOn = IUFindSwitch(ccdCooler, "COOLER_ON");
    ISwitch *coolerOff = IUFindSwitch(ccdCooler, "COOLER_OFF");
    
    if (!coolerOn || !coolerOff)
    {
        logStream(MESSAGE_ERROR) << "Cooler switches not found" << sendLog;
        return -1;
    }
    
    IUResetSwitch(ccdCooler);
    
    if (cooling)
        coolerOn->s = ISS_ON;
    else
        coolerOff->s = ISS_ON;
    
    if (!sendNewSwitch(ccdCooler))
    {
        logStream(MESSAGE_ERROR) << "Failed to switch cooler" << sendLog;
        return -1;
    }
    
    logStream(MESSAGE_INFO) << "Cooler " << (cooling ? "enabled" : "disabled") << sendLog;
    
    return 0;
}

// INDI Client Callbacks

void IndiCam::newDevice(INDI::BaseDevice *dp)
{
    if (dp->getDeviceName() == cameraDevice)
    {
        logStream(MESSAGE_INFO) << "Camera device found: " << cameraDevice << sendLog;
        camera = dp;
	watchDevice(cameraDevice.c_str());
	setBLOBMode(B_ALSO, cameraDevice.c_str(), "CCD1");   // sometimes "CCD1" is the property
	// or "CCD1" property with element "CCD1" depending on driver.
    }
}

void IndiCam::removeDevice(INDI::BaseDevice *dp)
{
    if (dp == camera)
    {
        logStream(MESSAGE_WARNING) << "Camera device removed" << sendLog;
        camera = nullptr;
        deviceConnected = false;
        indiConnected->setValueBool(false);
    }
}

void IndiCam::newProperty(INDI::Property property)
{
    if (!camera || property.getDeviceName() != cameraDevice)
        return;
    
    std::string propName = property.getName();
    
    if (propName == "CONNECTION")
        ccdConnection = property.getSwitch();
    else if (propName == "CCD_EXPOSURE")
        ccdExposure = property.getNumber();
    else if (propName == "CCD_TEMPERATURE")
        ccdTemperature = property.getNumber();
    else if (propName == "CCD_INFO")
        ccdInfo = property.getNumber();
    else if (propName == "CCD_BINNING")
        ccdBinning = property.getNumber();
    else if (propName == "CCD_FRAME")
        ccdFrame = property.getNumber();
    else if (propName == "CCD_COOLER")
        ccdCooler = property.getSwitch();
    else if (propName == "CCD1")
        ccdImage = property.getBLOB();
}

void IndiCam::removeProperty(INDI::Property property)
{
    // Handle property removal if needed
}

void IndiCam::newBLOB(IBLOB *bp)
{
    if (!bp || strcmp(bp->name, "CCD1") != 0)
        return;
    
    logStream(MESSAGE_INFO) << "Image received: " << bp->size << " bytes, format: " 
                           << bp->format << sendLog;
    
    processImage(bp);
}

void IndiCam::newSwitch(ISwitchVectorProperty *svp)
{
    if (svp->device == cameraDevice && strcmp(svp->name, "CONNECTION") == 0)
    {
        ISwitch *conn = IUFindSwitch(svp, "CONNECT");
        if (conn && conn->s == ISS_ON)
        {
            deviceConnected = true;
            indiConnected->setValueBool(true);
            logStream(MESSAGE_INFO) << "Camera connected" << sendLog;
        }
        else
        {
            deviceConnected = false;
            indiConnected->setValueBool(false);
            logStream(MESSAGE_INFO) << "Camera disconnected" << sendLog;
        }
    }
}

void IndiCam::newNumber(INumberVectorProperty *nvp)
{
    if (nvp->device == cameraDevice && strcmp(nvp->name, "CCD_TEMPERATURE") == 0)
    {
        INumber *temp = IUFindNumber(nvp, "CCD_TEMPERATURE_VALUE");
        if (temp)
        {
            lastTemp = temp->value;
        }
    }
}

void IndiCam::newText(ITextVectorProperty *tvp)
{
    // Handle text properties if needed
}

void IndiCam::newLight(ILightVectorProperty *lvp)
{
    // Handle light properties if needed
}

void IndiCam::newMessage(INDI::BaseDevice *dp, int messageID)
{
    if (dp == camera)
    {
        logStream(MESSAGE_INFO) << "INDI message from camera: " << dp->messageQueue(messageID) << sendLog;
    }
}

void IndiCam::serverConnected()
{
    logStream(MESSAGE_INFO) << "Connected to INDI server" << sendLog;
}

void IndiCam::serverDisconnected(int exit_code)
{
    logStream(MESSAGE_WARNING) << "Disconnected from INDI server, exit code: " << exit_code << sendLog;
    deviceConnected = false;
    camera = nullptr;
    indiConnected->setValueBool(false);
}

// Helper Methods

bool IndiCam::connectToIndiServer()
{
    setServer(indiServer.c_str(), indiPort);
    
    if (!connectServer())
    {
        return false;
    }
    
    logStream(MESSAGE_INFO) << "Connected to INDI server at " << indiServer << ":" << indiPort << sendLog;
    
    return true;
}

bool IndiCam::connectCamera()
{
    if (!ccdConnection)
    {
        logStream(MESSAGE_ERROR) << "CONNECTION property not available" << sendLog;
        return false;
    }
    
    ISwitch *conn = IUFindSwitch(ccdConnection, "CONNECT");
    if (!conn)
    {
        logStream(MESSAGE_ERROR) << "CONNECT switch not found" << sendLog;
        return false;
    }
    
    IUResetSwitch(ccdConnection);
    conn->s = ISS_ON;
    
    if (!sendNewSwitch(ccdConnection))
    {
        logStream(MESSAGE_ERROR) << "Failed to send connect command" << sendLog;
        return false;
    }
    
    // Wait for connection
    int timeout = 10;
    while (timeout > 0 && !deviceConnected)
    {
        sleep(1);
        timeout--;
    }
    
    return deviceConnected;
}

bool IndiCam::disconnectCamera()
{
    if (!ccdConnection || !deviceConnected)
        return true;
    
    ISwitch *disconn = IUFindSwitch(ccdConnection, "DISCONNECT");
    if (!disconn)
        return false;
    
    IUResetSwitch(ccdConnection);
    disconn->s = ISS_ON;
    
    sendNewSwitch(ccdConnection);
    
    deviceConnected = false;
    
    return true;
}

INDI::BaseDevice *IndiCam::getDevice(const char *name)
{
    // BaseClient doesn't have a getDevice method in older versions
    // We'll use our cached camera pointer if name matches
    if (camera && camera->getDeviceName() == name)
        return camera;
    return nullptr;
}

bool IndiCam::waitForProperty(const char *device, const char *property, int timeout)
{
    while (timeout > 0)
    {
        INDI::BaseDevice *dev = getDevice(device);
        if (dev)
        {
            INDI::Property prop = dev->getProperty(property);
            if (prop.isValid())
                return true;
        }
        
        sleep(1);
        timeout--;
    }
    
    return false;
}

void IndiCam::updateCameraInfo()
{
    if (!ccdInfo)
        return;
    
    INumber *w = IUFindNumber(ccdInfo, "CCD_MAX_X");
    INumber *h = IUFindNumber(ccdInfo, "CCD_MAX_Y");
    INumber *pixelX = IUFindNumber(ccdInfo, "CCD_PIXEL_SIZE");
    INumber *bpp = IUFindNumber(ccdInfo, "CCD_BITSPERPIXEL");
    
    if (w && h)
    {
        logStream(MESSAGE_INFO) << "Camera resolution: " << (int)w->value << "x" << (int)h->value << sendLog;
    }
    
    if (pixelX)
    {
        logStream(MESSAGE_INFO) << "Pixel size: " << pixelX->value << " µm" << sendLog;
    }
    
    if (bpp)
    {
        logStream(MESSAGE_INFO) << "Bits per pixel: " << (int)bpp->value << sendLog;
    }
}

void IndiCam::processImage(IBLOB *bp)
{
    if (!bp || !bp->blob || bp->size <= 0)
        return;

    fitsfile *fptr = nullptr;
    int status = 0;

    size_t blobSize = static_cast<size_t>(bp->size);

    if (fits_open_memfile(&fptr,
                  "memfits",
                  READONLY,
                  reinterpret_cast<void **>(&bp->blob),
                  &blobSize,
                  0,
                  nullptr,
                  &status))
    {
        logStream(MESSAGE_ERROR) << "FITS open failed" << sendLog;
        return;
    }

    int bitpix = 0, naxis = 0;
    long naxes[3] = {0, 0, 0};

    if (fits_get_img_param(fptr, 3, &bitpix, &naxis, naxes, &status) ||
        naxis != 3 || naxes[2] != 3 || bitpix != 16)
    {
        fits_close_file(fptr, &status);
        logStream(MESSAGE_ERROR) << "Unsupported FITS format" << sendLog;
        return;
    }

    const int width  = naxes[0];
    const int height = naxes[1];
    const size_t planePixels = width * height;

    std::vector<int16_t> r(planePixels), g(planePixels), b(planePixels);

    long fpixel[3] = {1, 1, 1};
    fits_read_pix(fptr, TSHORT, fpixel, planePixels, nullptr, r.data(), nullptr, &status);
    fpixel[2] = 2;
    fits_read_pix(fptr, TSHORT, fpixel, planePixels, nullptr, g.data(), nullptr, &status);
    fpixel[2] = 3;
    fits_read_pix(fptr, TSHORT, fpixel, planePixels, nullptr, b.data(), nullptr, &status);

    fits_close_file(fptr, &status);

    if (status)
    {
        logStream(MESSAGE_ERROR) << "FITS read failed" << sendLog;
        return;
    }

    uint16_t *dst = reinterpret_cast<uint16_t *>(getDataBuffer(0));

    for (size_t i = 0; i < planePixels; ++i)
    {
        dst[3*i + 0] = r[i] + 32768;
        dst[3*i + 1] = g[i] + 32768;
        dst[3*i + 2] = b[i] + 32768;
    }

    exposureInProgress = false;
    imageReady = true;

    logStream(MESSAGE_INFO)
        << "RGB image received: " << width << "x" << height
        << " interleaved RGB" << sendLog;
}

bool IndiCam::sendNewNumber(INumberVectorProperty *nvp)
{
    if (!nvp)
        return false;
    
    INDI::BaseClient::sendNewNumber(nvp);
    return true;
}

bool IndiCam::sendNewSwitch(ISwitchVectorProperty *svp)
{
    if (!svp)
        return false;
    
    INDI::BaseClient::sendNewSwitch(svp);
    return true;
}

// Main entry point
int main(int argc, char **argv)
{
    IndiCam device(argc, argv);
    return device.run();
}
