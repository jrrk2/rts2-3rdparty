/*
 * Celestron Origin Camera Driver for RTS2 - Connect-on-Demand Version
 *
 * Key changes:
 *  - No persistent WebSocket connection
 *  - Connects only when starting exposure
 *  - Disconnects after image received
 *  - Uses short-lived connection model to avoid conflicts with telescope driver
 */

#include "origin-camd.h"
#include "origin-websocket.h"
#include "origin-data.h"

#include <sstream>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <sys/time.h>
#include <fitsio.h>

#include <fstream>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>

// Network headers
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <netdb.h>

#ifndef ORIGIN_CAM_VERBOSE
#define ORIGIN_CAM_VERBOSE 1
#endif

#if ORIGIN_CAM_VERBOSE
  #define VLOG(level) logStream(level) << "[ORIGIN_CAM] "
#else
  #define VLOG(level) if (false) logStream(level)
#endif

static inline double now_s()
{
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
}

static inline bool ends_with(const std::string& s, const std::string& suf)
{
    return s.size() >= suf.size() && s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
}

static inline bool isJPEG(const std::vector<uint8_t>& data)
{
    return data.size() >= 3 &&
           data[0] == 0xFF &&
           data[1] == 0xD8 &&
           data[2] == 0xFF;
}

#ifndef USEC_SEC
#define USEC_SEC 1000000L
#endif

// SimpleJSON class (same as before)
class SimpleJSON {
public:
    static std::map<std::string, std::string> parse(const std::string& json) {
        std::map<std::string, std::string> result;
        size_t pos = 0;
        while ((pos = json.find("\"", pos)) != std::string::npos) {
            size_t keyStart = pos + 1;
            size_t keyEnd = json.find("\"", keyStart);
            if (keyEnd == std::string::npos) break;

            std::string key = json.substr(keyStart, keyEnd - keyStart);

            size_t colonPos = json.find(":", keyEnd);
            if (colonPos == std::string::npos) break;

            size_t valueStart = colonPos + 1;
            while (valueStart < json.length() &&
                   (json[valueStart] == ' ' || json[valueStart] == '\t')) {
                valueStart++;
            }

            std::string value;
            if (valueStart < json.size() && json[valueStart] == '\"') {
                size_t valueEnd = json.find("\"", valueStart + 1);
                if (valueEnd != std::string::npos) {
                    value = json.substr(valueStart + 1, valueEnd - valueStart - 1);
                    pos = valueEnd + 1;
                } else {
                    break;
                }
            } else {
                size_t valueEnd = json.find_first_of(",}", valueStart);
                if (valueEnd != std::string::npos) {
                    value = json.substr(valueStart, valueEnd - valueStart);
                    size_t end = value.find_last_not_of(" \t\r\n");
                    if (end != std::string::npos)
                        value = value.substr(0, end + 1);
                    else
                        value.clear();
                    pos = valueEnd;
                } else {
                    break;
                }
            }

            result[key] = value;
        }
        return result;
    }

    static std::string create(const std::map<std::string, std::string>& data) {
        std::ostringstream json;
        json << "{";
        bool first = true;
        for (const auto& pair : data) {
            if (!first) json << ",";
            json << "\"" << pair.first << "\":";

            bool isNumeric = true;
            for (char c : pair.second) {
                if (!isdigit(c) && c != '.' && c != '-' && c != 'e' && c != 'E') {
                    isNumeric = false;
                    break;
                }
            }

            if (pair.second == "true" || pair.second == "false") {
                json << pair.second;
            } else if (isNumeric && !pair.second.empty()) {
                json << pair.second;
            } else {
                json << "\"" << pair.second << "\"";
            }
            first = false;
        }
        json << "}";
        return json.str();
    }
};

using namespace rts2camd;

OriginCam::OriginCam(int argc, char **argv)
    : Camera(argc, argv)
{
    telescopeHost = "";
    telescopePort = 80;
    useDiscovery = false;

    webSocket = nullptr;
    connected = false;

    cameraStatus = new CameraStatus();
    nextSequenceId = 1;

    exposureStart = 0;
    exposureDuration = 0;
    exposureInProgress = false;

    snapshotArmed = false;
    snapshotReady = false;
    snapshotArmTime = 0.0;

    imageBuffer = nullptr;
    imageBufferSize = 0;

    pendingImageRA = 0;
    pendingImageDec = 0;

    discoverySocket = -1;
    discovering = false;
    discoveryStartTime = 0;

    createValue(telescopeAddress, "telescope_address", "telescope IP address", false);
    telescopeAddress->setValueCharArr("");

    createValue(gain, "GAIN", "camera ISO/gain", true, RTS2_VALUE_WRITABLE);
    gain->setValueInteger(100);

    createValue(previewMode, "PREVIEW", "preview mode", true, RTS2_VALUE_WRITABLE);
    previewMode->setValueBool(false);

    createValue(ccdTemp, "CCD_TEMP", "CCD temperature", false);
    ccdTemp->setValueDouble(20);

    addOption('a', "address", 1, "telescope IP address (optional if using discovery)");
    addOption('p', "port", 1, "telescope port (default: 80)");
    addOption('D', "discover", 0, "auto-discover telescope on network");

    setDeviceName("ORIGIN_CAM");
}

OriginCam::~OriginCam()
{
    stopDiscovery();
    disconnectFromTelescope();
    delete cameraStatus;

    if (webSocket) {
        delete webSocket;
        webSocket = nullptr;
    }

    if (imageBuffer) {
        delete[] imageBuffer;
        imageBuffer = nullptr;
    }
}

int OriginCam::processOption(int opt)
{
    switch (opt)
    {
        case 'a':
            telescopeHost = std::string(optarg);
            telescopeAddress->setValueCharArr(telescopeHost.c_str());
            return 0;

        case 'p':
            telescopePort = atoi(optarg);
            return 0;

        case 'D':
            useDiscovery = true;
            return 0;

        default:
            return Camera::processOption(opt);
    }
}

int OriginCam::initHardware()
{
    if (telescopeHost.empty())
        useDiscovery = true;

    if (useDiscovery) {
        VLOG(MESSAGE_INFO) << "Starting telescope discovery..." << sendLog;

        if (!discoverTelescope()) {
            VLOG(MESSAGE_ERROR) << "Failed to discover telescope." << sendLog;
            return -1;
        }

        VLOG(MESSAGE_INFO) << "Discovered telescope at " << telescopeHost << sendLog;
        telescopeAddress->setValueCharArr(telescopeHost.c_str());
    }

    if (telescopeHost.empty()) {
        VLOG(MESSAGE_ERROR) << "Telescope IP address not specified." << sendLog;
        return -1;
    }

    // Don't connect here - connect on demand
    VLOG(MESSAGE_INFO) << "Origin camera initialized (connect-on-demand mode)" << sendLog;

    initChips();
    return 0;
}

int OriginCam::initChips()
{
    const int width = 3056;
    const int height = 2048;
    const float pixelSize = 3.76f;

    initCameraChip(width, height, pixelSize, pixelSize);
    initBinnings();

    if (dataChannels) {
        dataChannels->setValueInteger(1);
        sendValueAll(dataChannels);
    }

    VLOG(MESSAGE_INFO) << "Chip initialized: " << width << "x" << height << sendLog;

    return 0;
}

int OriginCam::initValues()
{
    return Camera::initValues();
}

int OriginCam::info()
{
    if (cameraStatus && cameraStatus->lastUpdate > 0) {
        ccdTemp->setValueDouble(cameraStatus->temperature);
    }

    return Camera::info();
}

// REVISED: Connect → Expose → Wait → Disconnect
int OriginCam::startExposure()
{
    VLOG(MESSAGE_INFO) << "startExposure() - connecting to telescope..." << sendLog;

    if (exposureInProgress) {
        VLOG(MESSAGE_ERROR) << "Exposure already in progress" << sendLog;
        return -1;
    }

    // Connect to telescope
    if (!connectToTelescope()) {
        VLOG(MESSAGE_ERROR) << "Failed to connect for exposure" << sendLog;
        return -1;
    }

    exposureDuration = getExposure();

    // Arm snapshot latch
    {
        std::lock_guard<std::mutex> lock(snapshotMutex);
        snapshotArmed = true;
        snapshotReady = false;
        snapshotArmTime = now_s();
        pendingImagePath.clear();
        pendingImageRA = 0.0;
        pendingImageDec = 0.0;
    }

    // Build capture command
    std::ostringstream paramsJson;
    paramsJson << "{"
               << "\"Exposure\":" << exposureDuration << ","
               << "\"ISO\":" << gain->getValueInteger() << ","
               << "\"Binning\":" << binningHorizontal() << ","
               << "\"ImageType\":\"Full\""
               << "}";

    VLOG(MESSAGE_INFO) << "Sending Snapshot: Exposure=" << exposureDuration
                       << " ISO=" << gain->getValueInteger() << sendLog;

    if (!sendCommand("RunSampleCapture", "TaskController", paramsJson.str())) {
        VLOG(MESSAGE_ERROR) << "sendCommand(Snapshot) failed" << sendLog;
        std::lock_guard<std::mutex> lock(snapshotMutex);
        snapshotArmed = false;
        snapshotReady = false;
        disconnectFromTelescope();
        return -1;
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);
    exposureStart = tv.tv_sec + tv.tv_usec / 1000000.0;
    exposureInProgress = true;

    return 0;
}

int OriginCam::stopExposure()
{
    VLOG(MESSAGE_INFO) << "stopExposure() called" << sendLog;

    if (!exposureInProgress) {
        return 0;
    }

    if (connected) {
        sendCommand("Abort", "Camera");
    }

    exposureInProgress = false;

    {
        std::lock_guard<std::mutex> lock(snapshotMutex);
        snapshotArmed = false;
        snapshotReady = false;
    }

    disconnectFromTelescope();

    VLOG(MESSAGE_INFO) << "Exposure aborted" << sendLog;
    return 0;
}

long OriginCam::isExposing()
{
    if (!exposureInProgress) {
        return -2;
    }

    // Check if TIFF is ready
    {
        std::lock_guard<std::mutex> lock(snapshotMutex);
        if (snapshotReady) {
            VLOG(MESSAGE_DEBUG) << "isExposing(): snapshotReady" << sendLog;
            return -2;
        }
    }

    // Poll WebSocket for messages while waiting
    if (webSocket && webSocket->isConnected()) {
        while (webSocket->hasData()) {
            std::string msg = webSocket->receiveText();
            if (!msg.empty()) {
                processMessage(msg);
            }
        }
    }

    // Check remaining exposure time
    struct timeval tv;
    gettimeofday(&tv, NULL);
    const double currentTime = tv.tv_sec + tv.tv_usec / 1000000.0;
    const double remaining = (exposureStart + exposureDuration) - currentTime;

    if (remaining > 0) {
        const long usec = (long)(remaining * USEC_SEC);
        return (usec > 0 ? usec : (USEC_SEC / 10));
    }

    // Keep polling until TIFF arrives
    return USEC_SEC / 10;
}

int OriginCam::doReadout()
{
    VLOG(MESSAGE_DEBUG) << "doReadout() called" << sendLog;

    bool hasSnapshot = false;
    {
        std::lock_guard<std::mutex> lock(snapshotMutex);
        hasSnapshot = snapshotReady;

        if (!hasSnapshot) {
            VLOG(MESSAGE_ERROR) << "doReadout(): no snapshot ready" << sendLog;
            snapshotArmed = false;
            snapshotReady = false;
            disconnectFromTelescope();
            return -1;
        }

        snapshotReady = false;
        snapshotArmed = false;
    }

    exposureInProgress = false;

    // Disconnect AFTER receiving image
    disconnectFromTelescope();

    if (monoFrame.empty()) {
        VLOG(MESSAGE_ERROR) << "doReadout(): monoFrame is empty" << sendLog;
        return -1;
    }

    // Generate filename
    time_t now = time(nullptr);
    struct tm *tm_info = gmtime(&now);
    char filename[256];
    snprintf(filename, sizeof(filename),
             "/home/jonathan/data/images/ORIGIN_CAM_%04d%02d%02d_%02d%02d%02d.fits",
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);

    if (!writeFITSforRTS2(filename)) {
        VLOG(MESSAGE_ERROR) << "Failed to write FITS: " << filename << sendLog;
        return -1;
    }

    VLOG(MESSAGE_INFO) << "Calling fitsDataTransfer for: " << filename << sendLog;
    int ret = fitsDataTransfer(filename);
    VLOG(MESSAGE_INFO) << "fitsDataTransfer returned: " << ret << sendLog;

    return -2;
}

bool OriginCam::writeFITSforRTS2(const char *filename)
{
    fitsfile *fptr;
    int status = 0;
    long naxes[2] = {3056, 2048};

    fits_create_file(&fptr, filename, &status);
    if (status) {
        VLOG(MESSAGE_ERROR) << "fits_create_file failed: " << status << sendLog;
        return false;
    }

    fits_create_img(fptr, USHORT_IMG, 2, naxes, &status);
    if (status) {
        VLOG(MESSAGE_ERROR) << "fits_create_img failed: " << status << sendLog;
        fits_close_file(fptr, &status);
        return false;
    }

    fits_write_img(fptr, TUSHORT, 1, monoFrame.size(),
                   (uint16_t*)monoFrame.data(), &status);
    if (status) {
        VLOG(MESSAGE_ERROR) << "fits_write_img failed: " << status << sendLog;
        fits_close_file(fptr, &status);
        return false;
    }

    // Add keywords
    char instrume[] = "ORIGIN_CAM";
    fits_update_key(fptr, TSTRING, "INSTRUME", instrume, "Camera name", &status);

    double exptime = exposureDuration;
    fits_update_key(fptr, TDOUBLE, "EXPTIME", &exptime, "Exposure time", &status);

    int gainval = gain->getValueInteger();
    fits_update_key(fptr, TINT, "GAIN", &gainval, "ISO/Gain", &status);

    int binx = binningHorizontal();
    int biny = binningVertical();
    fits_update_key(fptr, TINT, "XBINNING", &binx, "Horizontal binning", &status);
    fits_update_key(fptr, TINT, "YBINNING", &biny, "Vertical binning", &status);

    char dateobs[32];
    time_t now_time = time(nullptr);
    struct tm *tm_info = gmtime(&now_time);
    snprintf(dateobs, sizeof(dateobs), "%04d-%02d-%02dT%02d:%02d:%02d",
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
    fits_update_key(fptr, TSTRING, "DATE-OBS", dateobs, "UTC date/time", &status);

    fits_close_file(fptr, &status);

    if (status) {
        VLOG(MESSAGE_ERROR) << "FITS write errors: " << status << sendLog;
        return false;
    }

    VLOG(MESSAGE_INFO) << "FITS file written: " << filename << sendLog;
    return true;
}

int OriginCam::setCoolTemp(float new_temp)
{
    (void)new_temp;
    return -1;
}

int OriginCam::switchCooling(bool cooling)
{
    (void)cooling;
    return -1;
}

// ---------- REVISED: Short-lived connection ----------

bool OriginCam::connectToTelescope()
{
    if (webSocket) {
        delete webSocket;
        webSocket = nullptr;
    }

    webSocket = new OriginWebSocket();

    VLOG(MESSAGE_INFO) << "Connecting WebSocket to " << telescopeHost << sendLog;

    if (!webSocket->connect(telescopeHost, telescopePort, "/SmartScope-1.0/mountControlEndpoint")) {
        VLOG(MESSAGE_ERROR) << "WebSocket connection failed" << sendLog;
        delete webSocket;
        webSocket = nullptr;
        return false;
    }

    connected = true;
    VLOG(MESSAGE_INFO) << "Connected to camera via WebSocket" << sendLog;
    return true;
}

void OriginCam::disconnectFromTelescope()
{
    if (webSocket) {
        webSocket->disconnect();
        delete webSocket;
        webSocket = nullptr;
    }

    connected = false;
    VLOG(MESSAGE_INFO) << "Disconnected from telescope" << sendLog;
}

bool OriginCam::sendCommand(const std::string& command,
                            const std::string& destination,
                            const std::string& params)
{
    if (!webSocket || !webSocket->isConnected()) {
        VLOG(MESSAGE_ERROR) << "sendCommand: not connected" << sendLog;
        return false;
    }

    std::map<std::string, std::string> cmdData;
    cmdData["Command"]     = command;
    cmdData["Destination"] = destination;
    cmdData["SequenceID"]  = std::to_string(nextSequenceId++);
    cmdData["Source"]      = "RTS2";
    cmdData["Type"]        = "Command";

    std::string message = SimpleJSON::create(cmdData);

    if (!params.empty())
    {
        if (!message.empty() && message.back() == '}')
            message.pop_back();

        std::string p = params;
        if (!p.empty() && p.front() == '{')
            p.erase(p.begin());

        message += ",";
        message += p;
    }

    VLOG(MESSAGE_DEBUG) << "WS SEND: " << message << sendLog;

    return webSocket->sendText(message);
}

void OriginCam::processMessage(const std::string& message)
{
    auto data = SimpleJSON::parse(message);

    const std::string source  = (data.count("Source")  ? data["Source"]  : "");
    const std::string command = (data.count("Command") ? data["Command"] : "");
    const std::string type    = (data.count("Type")    ? data["Type"]    : "");

    if (type != "Notification" && type != "Response") {
        return;
    }

    if (source == "Camera") {
        updateCameraStatus(message);
        return;
    }

    if (source == "ImageServer" && command == "NewImageReady") {
        const std::string filePath = (data.count("FileLocation") ? data["FileLocation"] : "");
        double ra = 0.0, dec = 0.0;
        try { if (data.count("Ra"))  ra  = std::stod(data["Ra"]); }  catch (...) {}
        try { if (data.count("Dec")) dec = std::stod(data["Dec"]); } catch (...) {}

        handleImageReady(filePath, ra, dec);
        return;
    }
}

void OriginCam::updateCameraStatus(const std::string& jsonData)
{
    auto data = SimpleJSON::parse(jsonData);

    if (data.find("ISO") != data.end()) {
        try { cameraStatus->iso = std::stoi(data["ISO"]); } catch (...) {}
    }
    if (data.find("Exposure") != data.end()) {
        try { cameraStatus->exposure = std::stod(data["Exposure"]); } catch (...) {}
    }
    if (data.find("Binning") != data.end()) {
        try { cameraStatus->binning = std::stoi(data["Binning"]); } catch (...) {}
    }
    if (data.find("CameraTemperature") != data.end()) {
        try { cameraStatus->temperature = std::stod(data["CameraTemperature"]); } catch (...) {}
    }

    cameraStatus->lastUpdate = time(nullptr);
}

void OriginCam::handleImageReady(const std::string& filePath, double ra, double dec)
{
    if (filePath.empty())
        return;

    const bool isJpg  = ends_with(filePath, ".jpg")  || ends_with(filePath, ".jpeg");
    const bool isTiff = ends_with(filePath, ".tif")  || ends_with(filePath, ".tiff");

    if (isJpg) {
        return;  // Ignore preview
    }

    if (!isTiff) {
        return;
    }

    // Check if armed
    {
        std::lock_guard<std::mutex> lock(snapshotMutex);
        if (!snapshotArmed) {
            VLOG(MESSAGE_INFO) << "TIFF arrived but not armed: " << filePath << sendLog;
            return;
        }
    }

    pendingImagePath = filePath;
    pendingImageRA   = ra;
    pendingImageDec  = dec;

    const std::string url = "http://" + telescopeHost + "/SmartScope-1.0/dev2/" + filePath;

    VLOG(MESSAGE_INFO) << "Downloading TIFF: " << filePath << sendLog;

    std::vector<uint8_t> imageData = downloadImageSync(url);

    if (imageData.empty()) {
        VLOG(MESSAGE_ERROR) << "TIFF download failed" << sendLog;
        return;
    }

    if (isJPEG(imageData)) {
        VLOG(MESSAGE_WARNING) << "Got JPEG for TIFF path (ignoring)" << sendLog;
        return;
    }

    if (!decodeTIFF(imageData)) {
        VLOG(MESSAGE_ERROR) << "TIFF decode failed" << sendLog;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(snapshotMutex);
        snapshotReady = true;
    }

    VLOG(MESSAGE_INFO) << "Snapshot TIFF ready: " << filePath << sendLog;
}

bool OriginCam::decodeTIFF(const std::vector<uint8_t> &imageData)
{
    const std::string tmp = "/tmp/origin_sample.tiff";
    {
        std::ofstream f(tmp, std::ios::binary);
        f.write(reinterpret_cast<const char*>(imageData.data()), imageData.size());
    }

    TIFF *tif = TIFFOpen(tmp.c_str(), "r");
    if (!tif)
        return false;

    uint32_t width = 0, height = 0;
    uint16_t spp = 0, bps = 0;

    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
    TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &spp);
    TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bps);

    if (width == 0 || height == 0 || spp != 3 || bps != 16) {
        TIFFClose(tif);
        return false;
    }

    monoFrame.resize((size_t)width * height);

    std::vector<uint16_t> scanline(width * 3);

    for (uint32_t y = 0; y < height; y++) {
        if (TIFFReadScanline(tif, scanline.data(), y) < 0) {
            TIFFClose(tif);
            return false;
        }

        for (uint32_t x = 0; x < width; x++) {
            uint16_t R = scanline[x * 3 + 0];
            uint16_t G = scanline[x * 3 + 1];
            uint16_t B = scanline[x * 3 + 2];

            uint16_t v;
            if ((y & 1) == 0)
                v = ((x & 1) == 0) ? R : G;
            else
                v = ((x & 1) == 0) ? G : B;

            monoFrame[(size_t)y * width + x] = v;
        }
    }

    TIFFClose(tif);
    return true;
}

// Discovery implementation (unchanged)
bool OriginCam::discoverTelescope()
{
    if (!startDiscovery()) {
        return false;
    }

    time_t startTime = time(nullptr);
    while (time(nullptr) - startTime < 30) {
        pollDiscovery();

        if (!telescopeHost.empty()) {
            stopDiscovery();
            return true;
        }

        usleep(100000);
    }

    stopDiscovery();
    return false;
}

bool OriginCam::startDiscovery()
{
    discoverySocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (discoverySocket < 0) {
        return false;
    }

    int flags = fcntl(discoverySocket, F_GETFL, 0);
    fcntl(discoverySocket, F_SETFL, flags | O_NONBLOCK);

    int reuse = 1;
    setsockopt(discoverySocket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    int broadcast = 1;
    setsockopt(discoverySocket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(55555);

    if (bind(discoverySocket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(discoverySocket);
        discoverySocket = -1;
        return false;
    }

    discovering = true;
    return true;
}

void OriginCam::stopDiscovery()
{
    if (discoverySocket >= 0) {
        close(discoverySocket);
        discoverySocket = -1;
    }
    discovering = false;
}

void OriginCam::pollDiscovery()
{
    if (!discovering || discoverySocket < 0) {
        return;
    }

    char buffer[4096];
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);

    while (discoverySocket >= 0) {
        ssize_t bytesRead = recvfrom(discoverySocket, buffer, sizeof(buffer) - 1, 0,
                                     (struct sockaddr*)&sender_addr, &sender_len);

        if (bytesRead < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            break;
        }

        if (bytesRead == 0) {
            break;
        }

        buffer[bytesRead] = '\0';
        std::string datagram(buffer, (size_t)bytesRead);

        char sender_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &sender_addr.sin_addr, sender_ip, INET_ADDRSTRLEN);

        if (datagram.find("Origin") != std::string::npos &&
            datagram.find("IP Address") != std::string::npos) {

            telescopeHost = sender_ip;
            return;
        }
    }
}

// HTTP download (unchanged)
std::vector<uint8_t> OriginCam::downloadImageSync(const std::string &url)
{
    std::vector<uint8_t> result;

    std::string proto = "http://";
    if (url.rfind(proto, 0) != 0)
        return result;

    std::string rest = url.substr(proto.size());
    std::string host;
    std::string path = "/";
    int port = 80;

    auto slashPos = rest.find('/');
    if (slashPos != std::string::npos)
    {
        host = rest.substr(0, slashPos);
        path = rest.substr(slashPos);
    }
    else
    {
        host = rest;
    }

    auto colonPos = host.find(':');
    if (colonPos != std::string::npos)
    {
        port = std::stoi(host.substr(colonPos + 1));
        host = host.substr(0, colonPos);
    }

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
        return result;

    struct timeval timeout;
    timeout.tv_sec = 60;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    struct hostent *server = gethostbyname(host.c_str());
    if (!server)
    {
        close(sock);
        return result;
    }

    struct sockaddr_in serv_addr {};
    serv_addr.sin_family = AF_INET;
    std::memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, (size_t)server->h_length);
    serv_addr.sin_port = htons((uint16_t)port);

    if (connect(sock, reinterpret_cast<struct sockaddr *>(&serv_addr), sizeof(serv_addr)) < 0)
    {
        close(sock);
        return result;
    }

    std::string request =
        "GET " + path + " HTTP/1.1\r\n"
        "Host: " + host + "\r\n"
        "Connection: close\r\n\r\n";

    if (send(sock, request.c_str(), request.size(), 0) < 0)
    {
        close(sock);
        return result;
    }

    std::vector<uint8_t> response;
    response.reserve(8 * 1024 * 1024);

    char buffer[65536];
    while (true)
    {
        ssize_t n = recv(sock, buffer, sizeof(buffer), 0);
        if (n > 0) {
            response.insert(response.end(), buffer, buffer + n);
        } else if (n == 0) {
            break;
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            break;
        }
    }

    close(sock);

    const std::string headerSep = "\r\n\r\n";
    auto it = std::search(response.begin(), response.end(),
                          headerSep.begin(), headerSep.end());

    if (it == response.end())
        return result;

    it += (ptrdiff_t)headerSep.size();
    result.assign(it, response.end());
    return result;
}

int OriginCam::commandAuthorized(rts2core::Connection *conn)
{
    if (conn->isCommand("getCommand"))
    {
        char *cmd;
        if (conn->paramNextString(&cmd) || !conn->paramEnd())
            return -2;

        if (!strcmp(cmd, "expose"))
            return 0;

        if (!strcmp(cmd, "stopexpo"))
            return 0;

        return -1;
    }

    return Camera::commandAuthorized(conn);
}

int main(int argc, char **argv)
{
    OriginCam device(argc, argv);
    return device.run();
}
