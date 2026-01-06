/*
 * Celestron Origin Camera Driver for RTS2
 * Drop-in replacement for origin-camd.cpp
 *
 * Key fixes vs previous version:
 *  - Uses SequenceID (not Sequence) in outbound messages
 *  - Treats preview JPEG stream as background noise (never completes exposure)
 *  - Arms a one-shot latch on Snapshot; completes exposure ONLY when a TIFF arrives
 *  - Moves download/decode/sendImage/endReadout into RTS2 thread (doReadout),
 *    not the WebSocket RX thread (thread-safety / RTS2 lifecycle correctness)
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

// Network headers for discovery
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>

// HTTP download headers
#include <netdb.h>
// --- ADD near top of origin-camd.cpp (after includes) ---
#include <mutex>
#include <chrono>

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

// --- ADD helper near top of origin-camd.cpp ---
static inline bool isJPEG(const std::vector<uint8_t>& data)
{
    // JPEG files start with: FF D8 FF
    return data.size() >= 3 &&
           data[0] == 0xFF &&
           data[1] == 0xD8 &&
           data[2] == 0xFF;
}

#ifndef USEC_SEC
#define USEC_SEC 1000000L
#endif

// Reuse the SimpleJSON class from telescope driver
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

static inline double nowSeconds()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1e6;
}

static inline bool ends_with_ci(const std::string& s, const std::string& suffix)
{
    if (s.size() < suffix.size()) return false;
    auto it = s.end() - (ptrdiff_t)suffix.size();
    for (size_t i = 0; i < suffix.size(); i++) {
        char a = (char)tolower((unsigned char)*(it + (ptrdiff_t)i));
        char b = (char)tolower((unsigned char)suffix[i]);
        if (a != b) return false;
    }
    return true;
}

static inline std::string basename_only(const std::string& p)
{
    auto slash = p.find_last_of('/');
    if (slash == std::string::npos) return p;
    return p.substr(slash + 1);
}

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

    // Snapshot latch state
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

    createValue(previewMode, "PREVIEW", "preview mode (faster downloads) (FYI: Origin preview stream is always on)", true, RTS2_VALUE_WRITABLE);
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

// --- REPLACE OriginCam::initHardware() ---
int OriginCam::initHardware()
{
    // If no address specified, use discovery
    if (telescopeHost.empty())
        useDiscovery = true;

    if (useDiscovery) {
        VLOG(MESSAGE_INFO) << "Starting telescope discovery..." << sendLog;

        if (!discoverTelescope()) {
            VLOG(MESSAGE_ERROR) << "Failed to discover telescope. You can specify IP with -a option." << sendLog;
            return -1;
        }

        VLOG(MESSAGE_INFO) << "Discovered telescope at " << telescopeHost << sendLog;
        telescopeAddress->setValueCharArr(telescopeHost.c_str());
    }

    if (telescopeHost.empty()) {
        VLOG(MESSAGE_ERROR) << "Telescope IP address not specified and discovery failed. Use -a option or -D for discovery." << sendLog;
        return -1;
    }

    if (!connectToTelescope()) {
        VLOG(MESSAGE_ERROR) << "Failed to connect to telescope at " << telescopeHost << ":" << telescopePort << sendLog;
        return -1;
    }

    VLOG(MESSAGE_INFO) << "Origin camera initialized successfully" << sendLog;

    // CRITICAL: ensure RTS2 sees a valid exposable chip
    initChips();
    return 0;
}

int OriginCam::initChips()
{
    // Celestron Origin has a Sony IMX410 sensor
    const int width = 3056;
    const int height = 2048;
    const float pixelSize = 3.76f;

    initCameraChip(width, height, pixelSize, pixelSize);
    initBinnings();

    // ✅ Explicitly set single data channel
    if (dataChannels) {
        dataChannels->setValueInteger(1);
        sendValueAll(dataChannels);
    }

    VLOG(MESSAGE_INFO) << "Chip initialized: " << width << "x" << height
                       << " pixels, " << pixelSize << "µm" << sendLog;

    return 0;
}

int OriginCam::initValues()
{
    // Call base class first
    int ret = Camera::initValues();
    if (ret)
        return ret;

    // ✅ For single-channel cameras, don't create dataChannels
    // The base class will handle this correctly with defaults

    VLOG(MESSAGE_INFO) << "initValues() completed (single-channel camera)" << sendLog;

    return 0;
}

// --- REPLACE OriginCam::pollLoop() ---
void OriginCam::pollLoop()
{
    VLOG(MESSAGE_INFO) << "pollLoop started" << sendLog;

    while (pollRunning)
    {
        if (webSocket && webSocket->isConnected())
        {
            int drained = 0;
            while (webSocket->hasData())
            {
                std::string msg = webSocket->receiveText();
                if (!msg.empty()) {
                    drained++;
                    processMessage(msg);
                }
            }

#if ORIGIN_CAM_VERBOSE
            if (drained > 0 && drained > 50) {
                VLOG(MESSAGE_DEBUG) << "pollLoop drained " << drained << " messages this tick" << sendLog;
            }
#endif
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    VLOG(MESSAGE_INFO) << "pollLoop exiting" << sendLog;
}

// --- REPLACE OriginCam::info() ---
// Adds helpful status logging and avoids spamming
int OriginCam::info()
{
    if (cameraStatus && cameraStatus->lastUpdate > 0) {
        // Many Origin firmwares call it CameraTemperature (not CCD_TEMP); we just reflect what we have.
        ccdTemp->setValueDouble(cameraStatus->temperature);
    }

#if ORIGIN_CAM_VERBOSE
    static time_t last = 0;
    time_t now = time(nullptr);
    if (now != last && (now % 5) == 0) {
        std::lock_guard<std::mutex> lock(snapshotMutex);
        VLOG(MESSAGE_INFO) << "status: connected=" << (connected ? "yes" : "no")
                           << " exposing=" << (exposureInProgress ? "yes" : "no")
                           << " armed=" << (snapshotArmed ? "yes" : "no")
                           << " ready=" << (snapshotReady ? "yes" : "no")
                           << " exp=" << exposureDuration
                           << " ISO=" << (gain ? gain->getValueInteger() : -1)
                           << sendLog;
        last = now;
    }
#endif

    return Camera::info();
}

// --- REPLACE OriginCam::startExposure() ---
// Fixes:
//  - Arms snapshot latch BEFORE sending command (so we can't miss the TIFF)
//  - Forces PREVIEW value true in UI but does NOT attempt to disable device preview (device ignores)
//  - Uses consistent JSON key naming (SequenceID etc. handled in sendCommand)
//  - Verbose logging
int OriginCam::startExposure()
{
    VLOG(MESSAGE_INFO) << "startExposure() called" << sendLog;

    if (exposureInProgress) {
        VLOG(MESSAGE_ERROR) << "Exposure already in progress" << sendLog;
        return -1;
    }

    exposureDuration = getExposure();

    // Arm snapshot latch first (race-proof)
    {
        std::lock_guard<std::mutex> lock(snapshotMutex);
        snapshotArmed = true;
        snapshotReady = false;
        snapshotArmTime = now_s();
        pendingImagePath.clear();
        pendingImageRA = 0.0;
        pendingImageDec = 0.0;
    }

    // Build capture command params
    std::ostringstream paramsJson;
    paramsJson << "{"
               << "\"Exposure\":" << exposureDuration << ","
               << "\"ISO\":" << gain->getValueInteger() << ","
               << "\"Binning\":" << binningHorizontal() << ","
               << "\"ImageType\":\"" << (previewMode->getValueBool() ? "Preview" : "Full") << "\""
               << "}";

    // NOTE: device preview is always on; ImageType here likely only affects snapshot type/size.
    VLOG(MESSAGE_INFO) << "Sending Snapshot: Exposure=" << exposureDuration
                       << " ISO=" << gain->getValueInteger()
                       << " Bin=" << binningHorizontal()
                       << " ImageType=" << (previewMode->getValueBool() ? "Preview" : "Full")
                       << " (device preview stream remains ON)" << sendLog;

    if (!sendCommand("RunSampleCapture", "TaskController", paramsJson.str())) {
        VLOG(MESSAGE_ERROR) << "sendCommand(Snapshot) failed" << sendLog;
        // Disarm latch on failure
        std::lock_guard<std::mutex> lock(snapshotMutex);
        snapshotArmed = false;
        snapshotReady = false;
        return -1;
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);
    exposureStart = tv.tv_sec + tv.tv_usec / 1000000.0;
    exposureInProgress = true;

    return 0;
}

// --- REPLACE OriginCam::stopExposure() ---
// Fix: also disarm snapshot latch
int OriginCam::stopExposure()
{
    VLOG(MESSAGE_INFO) << "stopExposure() called" << sendLog;

    if (!exposureInProgress) {
        return 0;
    }

    sendCommand("Abort", "Camera");

    exposureInProgress = false;

    {
        std::lock_guard<std::mutex> lock(snapshotMutex);
        snapshotArmed = false;
        snapshotReady = false;
    }

    VLOG(MESSAGE_INFO) << "Exposure aborted" << sendLog;
    return 0;
}

long OriginCam::isExposing()
{
    // If we are not in an exposure cycle, RTS2 expects -2 (idle)
    if (!exposureInProgress) {
        return -2;
    }

    // If TIFF is latched, signal completion to RTS2
    {
        std::lock_guard<std::mutex> lock(snapshotMutex);
        if (snapshotReady) {
            VLOG(MESSAGE_DEBUG) << "isExposing(): snapshotReady -> returning -2" << sendLog;
            return -2;  // This is correct - tells RTS2 to call doReadout()
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

    // Exposure time elapsed: keep RTS2 polling until TIFF arrives
    return USEC_SEC / 10; // 100ms
}

int OriginCam::doReadout()
{
    VLOG(MESSAGE_DEBUG) << "doReadout() called" << sendLog;

    bool hasSnapshot = false;
    {
        std::lock_guard<std::mutex> lock(snapshotMutex);
        hasSnapshot = snapshotReady;

        if (!hasSnapshot) {
            if (exposureInProgress) {
                VLOG(MESSAGE_ERROR)
                    << "doReadout(): called while exposure still in progress"
                    << sendLog;
                exposureInProgress = false;
            } else {
                VLOG(MESSAGE_ERROR)
                    << "doReadout(): called but no snapshot ready and no exposure running"
                    << sendLog;
            }

            snapshotArmed = false;
            snapshotReady = false;
            return -1;
        }

        snapshotReady = false;
        snapshotArmed = false;
    }

    exposureInProgress = false;

    // Create FITS filename
    time_t now = time(nullptr);
    struct tm *tm_info = gmtime(&now);
    char filename[256];
    snprintf(filename, sizeof(filename), 
             "/home/jonathan/data/images/ORIGIN_CAM_%04d%02d%02d_%02d%02d%02d.fits",
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);

    // Write FITS file directly
    if (!writeFITS(filename)) {
        VLOG(MESSAGE_ERROR) << "Failed to write FITS: " << filename << sendLog;
        return -1;
    }

    VLOG(MESSAGE_INFO) << "doReadout(): FITS file written to " << filename << sendLog;

    int ret = fitsDataTransfer(filename);
    VLOG(MESSAGE_INFO) << "fitsDataTransfer returned: " << ret << sendLog;

    return -2;
}

bool OriginCam::writeFITS(const char *filename)
{
    fitsfile *fptr;
    int status = 0;
    long naxis = 2;
    long naxes[2] = {3056, 2048};

    // Create FITS file
    fits_create_file(&fptr, filename, &status);
    if (status) {
        VLOG(MESSAGE_ERROR) << "fits_create_file failed: " << status << sendLog;
        return false;
    }

    // Create image HDU
    fits_create_img(fptr, USHORT_IMG, naxis, naxes, &status);
    if (status) {
        VLOG(MESSAGE_ERROR) << "fits_create_img failed: " << status << sendLog;
        fits_close_file(fptr, &status);
        return false;
    }

    // Write image data
    fits_write_img(fptr, TUSHORT, 1, monoFrame.size(), 
                   monoFrame.data(), &status);
    if (status) {
        VLOG(MESSAGE_ERROR) << "fits_write_img failed: " << status << sendLog;
        fits_close_file(fptr, &status);
        return false;
    }

    // Add basic keywords
    fits_update_key(fptr, TSTRING, "INSTRUME", (void*)"ORIGIN_CAM", 
                    "Camera name", &status);
    
    // ✅ Fix: store value in variable first
    double exptime = exposureDuration;
    fits_update_key(fptr, TDOUBLE, "EXPTIME", &exptime, 
                    "Exposure time", &status);
    
    // ✅ Fix: store gain in variable first
    int gainval = gain->getValueInteger();
    fits_update_key(fptr, TINT, "GAIN", &gainval, 
                    "ISO/Gain", &status);
    
    // Add more useful keywords
    int binx = binningHorizontal();
    int biny = binningVertical();
    fits_update_key(fptr, TINT, "XBINNING", &binx, "Horizontal binning", &status);
    fits_update_key(fptr, TINT, "YBINNING", &biny, "Vertical binning", &status);
    
    // Add timestamp
    char dateobs[32];
    time_t now = time(nullptr);
    struct tm *tm_info = gmtime(&now);
    snprintf(dateobs, sizeof(dateobs), "%04d-%02d-%02dT%02d:%02d:%02d",
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
    fits_update_key(fptr, TSTRING, "DATE-OBS", dateobs, "UTC date/time of observation", &status);

    // Close file
    fits_close_file(fptr, &status);

    if (status) {
        VLOG(MESSAGE_ERROR) << "fits_close_file failed: " << status << sendLog;
        return false;
    }

    return true;
}

int OriginCam::setCoolTemp(float new_temp)
{
    (void)new_temp;
    logStream(MESSAGE_WARNING) << "Temperature control not supported on Origin" << sendLog;
    return -1;
}

int OriginCam::switchCooling(bool cooling)
{
    (void)cooling;
    logStream(MESSAGE_WARNING) << "Cooling control not supported on Origin" << sendLog;
    return -1;
}

// -------------------- Private methods --------------------

// --- REPLACE OriginCam::connectToTelescope() ---
bool OriginCam::connectToTelescope()
{
    if (webSocket) {
        delete webSocket;
        webSocket = nullptr;
    }

    webSocket = new OriginWebSocket();

    VLOG(MESSAGE_INFO) << "Connecting WebSocket to "
                       << telescopeHost << ":" << telescopePort
                       << " path=/SmartScope-1.0/mountControlEndpoint" << sendLog;

    if (!webSocket->connect(telescopeHost, telescopePort, "/SmartScope-1.0/mountControlEndpoint")) {
        VLOG(MESSAGE_ERROR) << "WebSocket connection failed" << sendLog;
        delete webSocket;
        webSocket = nullptr;
        return false;
    }

    connected = true;

    // Good to query parameters once; device will still spam preview.
    sendCommand("GetCaptureParameters", "Camera");

    VLOG(MESSAGE_INFO) << "Connected to camera via WebSocket" << sendLog;

    pollRunning = true;
    pollThread = std::thread(&OriginCam::pollLoop, this);
    return true;
}

void OriginCam::disconnectFromTelescope()
{
    pollRunning = false;
    if (pollThread.joinable())
        pollThread.join();

    if (webSocket) {
        webSocket->disconnect();
        delete webSocket;
        webSocket = nullptr;
    }

    connected = false;
}

// --- REPLACE OriginCam::sendCommand(...) ---
// Fix: use SequenceID (matches mount driver + trace), and log full JSON
bool OriginCam::sendCommand(const std::string& command,
                            const std::string& destination,
                            const std::string& params)
{
    if (!webSocket || !webSocket->isConnected()) {
        VLOG(MESSAGE_ERROR) << "sendCommand(" << command << "): websocket not connected" << sendLog;
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
        // message ends with "}"
        if (!message.empty() && message.back() == '}')
            message.pop_back();

        // params begins with "{"
        std::string p = params;
        if (!p.empty() && p.front() == '{')
            p.erase(p.begin());

        message += ",";
        message += p; // ends with "}"
    }

    VLOG(MESSAGE_DEBUG) << "WS SEND: " << message << sendLog;

    return webSocket->sendText(message);
}

// --- REPLACE OriginCam::processMessage() ---
// Fixes:
//  - robust routing: accept Notifications/Responses, but NewImageReady is always spammy
//  - verbose logging for Snapshot window
void OriginCam::processMessage(const std::string& message)
{
    auto data = SimpleJSON::parse(message);

    const std::string source  = (data.count("Source")  ? data["Source"]  : "");
    const std::string command = (data.count("Command") ? data["Command"] : "");
    const std::string type    = (data.count("Type")    ? data["Type"]    : "");

    if (type != "Notification" && type != "Response") {
        return;
    }

#if ORIGIN_CAM_VERBOSE
    if (command == "NewImageReady" || command == "GetCaptureParameters" || command == "GetStatus") {
        // keep light; NewImageReady is very frequent
    } else {
        VLOG(MESSAGE_DEBUG) << "WS msg: Source=" << source << " Command=" << command << " Type=" << type << sendLog;
    }
#endif

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

// // --- REPLACE OriginCam::updateCameraStatus(...) ---
// Adds verbose logging + stores temperature if present in payload
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

    // Trace shows "CameraTemperature"
    if (data.find("CameraTemperature") != data.end()) {
        try { cameraStatus->temperature = std::stod(data["CameraTemperature"]); } catch (...) {}
    }

    cameraStatus->lastUpdate = time(nullptr);

#if ORIGIN_CAM_VERBOSE
    // Don't spam; log occasionally if you want:
    // VLOG(MESSAGE_DEBUG) << "CameraStatus updated (ISO=" << cameraStatus->iso
    //                    << " exp=" << cameraStatus->exposure
    //                    << " bin=" << cameraStatus->binning
    //                    << " temp=" << cameraStatus->temperature << ")" << sendLog;
#endif
}

// --- REPLACE OriginCam::handleImageReady() ---
// Fixes:
//  - Ignore all JPEG preview frames always
//  - Only accept TIFF when snapshotArmed is true and after snapshotArmTime
//  - When accepted, download and decode TIFF and set snapshotReady=true
void OriginCam::handleImageReady(const std::string& filePath, double ra, double dec)
{
    if (filePath.empty())
        return;

    const bool isJpg  = ends_with(filePath, ".jpg")  || ends_with(filePath, ".jpeg") || ends_with(filePath, ".JPG") || ends_with(filePath, ".JPEG");
    const bool isTiff = ends_with(filePath, ".tif")  || ends_with(filePath, ".tiff")|| ends_with(filePath, ".TIF") || ends_with(filePath, ".TIFF");

    // Always ignore preview JPEG spam at RTS2 boundary
    if (isJpg) {
#if ORIGIN_CAM_VERBOSE
        // keep this very quiet; uncomment if you want spam:
        // VLOG(MESSAGE_DEBUG) << "Ignoring preview JPEG: " << filePath << sendLog;
#endif
        return;
    }

    // If it's not TIFF/JPEG, ignore
    if (!isTiff) {
        VLOG(MESSAGE_DEBUG) << "Ignoring NewImageReady non-TIFF: " << filePath << sendLog;
        return;
    }

    // Decide if we should accept this TIFF as the snapshot frame
    double armTime = 0.0;
    bool armed = false;
    {
        std::lock_guard<std::mutex> lock(snapshotMutex);
        armed = snapshotArmed;
        armTime = snapshotArmTime;
        if (!armed) {
            VLOG(MESSAGE_INFO) << "TIFF arrived but snapshot not armed; ignoring: " << filePath << sendLog;
            return;
        }
    }

    const double t = now_s();
    if (t + 0.001 < armTime) { // should never happen, but be strict
        VLOG(MESSAGE_INFO) << "TIFF arrived before armTime (stale?); ignoring: " << filePath << sendLog;
        return;
    }

    pendingImagePath = filePath;
    pendingImageRA   = ra;
    pendingImageDec  = dec;

    const std::string url = "http://" + telescopeHost + "/SmartScope-1.0/dev2/" + filePath;

    VLOG(MESSAGE_INFO) << "Snapshot TIFF candidate: " << filePath
                       << " RA=" << ra << " Dec=" << dec
                       << " url=" << url << sendLog;

    std::vector<uint8_t> imageData = downloadImageSync(url);

    if (imageData.empty()) {
        VLOG(MESSAGE_ERROR) << "TIFF download failed for " << filePath << sendLog;
        // keep exposureInProgress true; RTS2 will continue to poll isExposing()
        return;
    }

    // If server mistakenly delivers JPEG payload at TIFF path, reject
    if (isJPEG(imageData)) {
        VLOG(MESSAGE_WARNING) << "Received JPEG payload for supposed TIFF " << filePath << " (ignoring)" << sendLog;
        return;
    }

    if (!decodeTIFF(imageData)) {
        VLOG(MESSAGE_ERROR) << "TIFF decode failed for " << filePath << sendLog;
        return;
    }

    // Mark snapshot ready for RTS2 state machine
    {
        std::lock_guard<std::mutex> lock(snapshotMutex);
        snapshotReady = true;
        // keep armed true until doReadout clears it (safer)
    }

    VLOG(MESSAGE_INFO) << "Snapshot TIFF latched OK: " << filePath << sendLog;
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

    VLOG(MESSAGE_INFO)
        << "TIFF meta: " << width << "x" << height
        << " spp=" << spp << " bps=" << bps
        << sendLog;

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
                v = ((x & 1) == 0) ? R : G;   // RG
            else
                v = ((x & 1) == 0) ? G : B;   // GB

            monoFrame[(size_t)y * width + x] = v;
        }
    }

    TIFFClose(tif);
    return true;
}

// -------------------- Discovery implementation --------------------

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
    logStream(MESSAGE_ERROR) << "Discovery timeout after 30 seconds" << sendLog;
    return false;
}

bool OriginCam::startDiscovery()
{
    logStream(MESSAGE_INFO) << "Starting UDP discovery on port 55555..." << sendLog;

    if (discoverySocket >= 0) {
        close(discoverySocket);
        discoverySocket = -1;
    }

    discoverySocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (discoverySocket < 0) {
        logStream(MESSAGE_ERROR) << "Failed to create UDP socket: " << strerror(errno) << sendLog;
        return false;
    }

    int flags = fcntl(discoverySocket, F_GETFL, 0);
    fcntl(discoverySocket, F_SETFL, flags | O_NONBLOCK);

    int reuse = 1;
    setsockopt(discoverySocket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#ifdef SO_REUSEPORT
    setsockopt(discoverySocket, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif

    int broadcast = 1;
    setsockopt(discoverySocket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(55555);

    if (bind(discoverySocket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        logStream(MESSAGE_ERROR) << "Failed to bind to port 55555: " << strerror(errno) << sendLog;
        close(discoverySocket);
        discoverySocket = -1;
        return false;
    }

    discovering = true;
    discoveryStartTime = time(nullptr);
    logStream(MESSAGE_INFO) << "Listening for telescope broadcasts..." << sendLog;

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
            } else {
                logStream(MESSAGE_ERROR) << "recvfrom error: " << strerror(errno) << sendLog;
                break;
            }
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

            std::string extractedIP;
            size_t pos = 0;
            while ((pos = datagram.find('.', pos)) != std::string::npos) {
                size_t start = datagram.rfind(' ', pos);
                if (start == std::string::npos) start = 0;
                else start++;

                size_t end = datagram.find(' ', pos);
                if (end == std::string::npos) end = datagram.length();

                std::string candidate = datagram.substr(start, end - start);

                if (std::count(candidate.begin(), candidate.end(), '.') == 3) {
                    extractedIP = candidate;
                    break;
                }
                pos++;
            }

            if (extractedIP.empty()) {
                extractedIP = sender_ip;
            }

            logStream(MESSAGE_INFO) << "Discovered Celestron Origin at " << extractedIP << sendLog;
            telescopeHost = extractedIP;
            return;
        }
    }
}

// -------------------- HTTP download (unchanged, but kept self-contained) --------------------

std::vector<uint8_t>
OriginCam::downloadImageSync(const std::string &url)
{
    std::vector<uint8_t> result;

    // Parse URL: http://host[:port]/path
    std::string proto = "http://";
    if (url.rfind(proto, 0) != 0)
    {
        std::cerr << "Invalid URL (not http): " << url << std::endl;
        return result;
    }

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
    {
        perror("socket");
        return result;
    }

    struct timeval timeout;
    timeout.tv_sec = 60;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    struct hostent *server = gethostbyname(host.c_str());
    if (!server)
    {
        std::cerr << "gethostbyname failed for " << host << std::endl;
        close(sock);
        return result;
    }

    struct sockaddr_in serv_addr {};
    serv_addr.sin_family = AF_INET;
    std::memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, (size_t)server->h_length);
    serv_addr.sin_port = htons((uint16_t)port);

    if (connect(sock, reinterpret_cast<struct sockaddr *>(&serv_addr), sizeof(serv_addr)) < 0)
    {
        perror("connect");
        close(sock);
        return result;
    }

    std::string request =
        "GET " + path + " HTTP/1.1\r\n"
        "Host: " + host + "\r\n"
        "Connection: close\r\n\r\n";

    if (send(sock, request.c_str(), request.size(), 0) < 0)
    {
        perror("send");
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
                // timeout
                break;
            }
            perror("recv");
            break;
        }
    }

    close(sock);

    // Strip HTTP headers
    const std::string headerSep = "\r\n\r\n";
    auto it = std::search(response.begin(), response.end(),
                          headerSep.begin(), headerSep.end());

    if (it == response.end())
    {
        std::cerr << "HTTP header not found\n";
        return result;
    }

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

// Main entry point
int main(int argc, char **argv)
{
    OriginCam device(argc, argv);
    return device.run();
}


