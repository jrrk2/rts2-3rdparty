/*
 * Celestron Origin Camera Driver for RTS2
 */

#include "origin-camd.h"
#include "origin-websocket.h"
#include "origin-data.h"
#include <sstream>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <sys/time.h>
#include <curl/curl.h>
#include <fitsio.h>

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
            if (json[valueStart] == '\"') {
                size_t valueEnd = json.find("\"", valueStart + 1);
                if (valueEnd != std::string::npos) {
                    value = json.substr(valueStart + 1, valueEnd - valueStart - 1);
                    pos = valueEnd + 1;
                }
            } else {
                size_t valueEnd = json.find_first_of(",}", valueStart);
                if (valueEnd != std::string::npos) {
                    value = json.substr(valueStart, valueEnd - valueStart);
                    size_t end = value.find_last_not_of(" \t\r\n");
                    value = value.substr(0, end + 1);
                    pos = valueEnd;
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

// CURL callback for downloading images
static size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    size_t realsize = size * nmemb;
    std::string *mem = (std::string *)userp;
    mem->append((char *)contents, realsize);
    return realsize;
}

using namespace rts2camd;

OriginCam::OriginCam(int argc, char **argv)
    : Camera(argc, argv)
{
    telescopeHost = "";
    telescopePort = 80;
    webSocket = nullptr;
    connected = false;
    cameraStatus = new CameraStatus();
    nextSequenceId = 1;
    
    exposureStart = 0;
    exposureDuration = 0;
    exposureInProgress = false;
    imageReady = false;
    
    imageBuffer = nullptr;
    imageBufferSize = 0;
    pendingImageRA = 0;
    pendingImageDec = 0;
    
    createValue(telescopeAddress, "telescope_address", "telescope IP address", false);
    telescopeAddress->setValueCharArr("");
    
    createValue(gain, "GAIN", "camera ISO/gain", true, RTS2_VALUE_WRITABLE);
    gain->setValueInteger(800);
    
    createValue(previewMode, "PREVIEW", "preview mode (faster downloads)", true, RTS2_VALUE_WRITABLE);
    previewMode->setValueBool(false);
    
    createValue(ccdTemp, "CCD_TEMP", "CCD temperature", false);
    ccdTemp->setValueDouble(20);
    
    addOption('a', "address", 1, "telescope IP address (required)");
    addOption('p', "port", 1, "telescope port (default: 80)");
}

OriginCam::~OriginCam()
{
    disconnectFromTelescope();
    delete cameraStatus;
    if (webSocket) {
        delete webSocket;
    }
    if (imageBuffer) {
        delete[] imageBuffer;
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
            
        default:
            return Camera::processOption(opt);
    }
}

int OriginCam::initHardware()
{
    if (telescopeHost.empty()) {
        logStream(MESSAGE_ERROR) << "Telescope IP address not specified. Use -a option." << sendLog;
        return -1;
    }
    
    if (!connectToTelescope()) {
        logStream(MESSAGE_ERROR) << "Failed to connect to telescope at " 
                                << telescopeHost << ":" << telescopePort << sendLog;
        return -1;
    }
    
    logStream(MESSAGE_INFO) << "Origin camera initialized successfully" << sendLog;
    
    return 0;
}

int OriginCam::initChips()
{
    // Celestron Origin has a Sony IMX410 sensor
    // 6072 x 4042 pixels, 5.9µm pixel size
    int width = 6072;
    int height = 4042;
    float pixelSize = 5.9;
    
    setSize(width, height, 0, 0);
    initBinnings();
    
    logStream(MESSAGE_INFO) << "Chip initialized: " << width << "x" << height 
                           << " pixels, " << pixelSize << "µm" << sendLog;
    
    return 0;
}

int OriginCam::info()
{
    // Poll for new messages
    if (webSocket && webSocket->isConnected()) {
        while (webSocket->hasData()) {
            std::string message = webSocket->receiveText();
            if (!message.empty()) {
                processMessage(message);
            }
        }
    }
    
    // Update temperature
    if (cameraStatus->lastUpdate > 0) {
        ccdTemp->setValueDouble(cameraStatus->temperature);
    }
    
    // Check if exposure is complete
    if (exposureInProgress) {
        struct timeval now;
        gettimeofday(&now, NULL);
        double currentTime = now.tv_sec + now.tv_usec / 1000000.0;
        
        if (currentTime >= exposureStart + exposureDuration) {
            // Exposure time elapsed, but we need to wait for image
            if (!imageReady) {
                // Still waiting for image download
                logStream(MESSAGE_DEBUG) << "Waiting for image download..." << sendLog;
            }
        }
    }
    
    return Camera::info();
}

int OriginCam::startExposure()
{
    if (exposureInProgress) {
        logStream(MESSAGE_ERROR) << "Exposure already in progress" << sendLog;
        return -1;
    }
    
    exposureDuration = getExposure();
    
    // Build capture command
    std::map<std::string, std::string> params;
    params["Exposure"] = std::to_string(exposureDuration);
    params["ISO"] = std::to_string(gain->getValueInteger());
    params["Binning"] = std::to_string(binningHorizontal());
    params["ImageType"] = previewMode->getValueBool() ? "Preview" : "Full";
    
    std::ostringstream paramsJson;
    paramsJson << "{";
    paramsJson << "\"Exposure\":" << exposureDuration << ",";
    paramsJson << "\"ISO\":" << gain->getValueInteger() << ",";
    paramsJson << "\"Binning\":" << binningHorizontal() << ",";
    paramsJson << "\"ImageType\":\"" << (previewMode->getValueBool() ? "Preview" : "Full") << "\"";
    paramsJson << "}";
    
    if (!sendCommand("Snapshot", "Camera", paramsJson.str())) {
        return -1;
    }
    
    struct timeval now;
    gettimeofday(&now, NULL);
    exposureStart = now.tv_sec + now.tv_usec / 1000000.0;
    exposureInProgress = true;
    imageReady = false;
    
    logStream(MESSAGE_INFO) << "Exposure started: " << exposureDuration << "s at ISO " 
                           << gain->getValueInteger() << sendLog;
    
    return 0;
}

int OriginCam::stopExposure()
{
    if (!exposureInProgress) {
        return 0;
    }
    
    // Send abort command
    sendCommand("Abort", "Camera");
    
    exposureInProgress = false;
    imageReady = false;
    
    logStream(MESSAGE_INFO) << "Exposure aborted" << sendLog;
    
    return 0;
}

long OriginCam::isExposing()
{
    if (!exposureInProgress) {
        return -2;  // Not exposing
    }
    
    if (imageReady) {
        return -2;  // Exposure complete, image ready
    }
    
    struct timeval now;
    gettimeofday(&now, NULL);
    double currentTime = now.tv_sec + now.tv_usec / 1000000.0;
    double remaining = exposureStart + exposureDuration - currentTime;
    
    if (remaining > 0) {
        return (long)(remaining * USEC_SEC);
    }
    
    // Exposure time elapsed, waiting for image
    return 100000;  // Check again in 100ms
}

int OriginCam::doReadout()
{
    if (!imageReady) {
        logStream(MESSAGE_ERROR) << "No image ready for readout" << sendLog;
        return -1;
    }
    
    if (!imageBuffer || imageBufferSize == 0) {
        logStream(MESSAGE_ERROR) << "No image data available" << sendLog;
        return -1;
    }
    
    // Copy image data to RTS2 buffer (chip 0)
    int dataSize = getUsedWidth() * getUsedHeight() * sizeof(uint16_t);
    
    if ((size_t)dataSize > imageBufferSize) {
        logStream(MESSAGE_ERROR) << "Image size mismatch" << sendLog;
        return -1;
    }
    
    memcpy(getDataBuffer(0), imageBuffer, dataSize);
    
    // Clear state
    exposureInProgress = false;
    imageReady = false;
    
    logStream(MESSAGE_INFO) << "Image readout complete" << sendLog;
    
    return 0;
}

int OriginCam::setCoolTemp(float new_temp)
{
    (void)new_temp;  // Suppress unused warning
    logStream(MESSAGE_WARNING) << "Temperature control not supported on Origin" << sendLog;
    return -1;
}

int OriginCam::switchCooling(bool cooling)
{
    (void)cooling;  // Suppress unused warning
    logStream(MESSAGE_WARNING) << "Cooling control not supported on Origin" << sendLog;
    return -1;
}

// Private methods

bool OriginCam::connectToTelescope()
{
    if (webSocket) {
        delete webSocket;
    }
    
    webSocket = new OriginWebSocket();
    
    if (!webSocket->connect(telescopeHost, telescopePort, "/cgi-bin/AlpacaWebSocket")) {
        logStream(MESSAGE_ERROR) << "WebSocket connection failed" << sendLog;
        delete webSocket;
        webSocket = nullptr;
        return false;
    }
    
    connected = true;
    
    // Send initial status request
    sendCommand("GetCaptureParameters", "Camera");
    
    logStream(MESSAGE_INFO) << "Connected to camera via WebSocket" << sendLog;
    
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
}

bool OriginCam::sendCommand(const std::string& command, const std::string& destination,
                            const std::string& params)
{
    if (!webSocket || !webSocket->isConnected()) {
        return false;
    }
    
    std::map<std::string, std::string> cmdData;
    cmdData["Sequence"] = std::to_string(nextSequenceId++);
    cmdData["Source"] = "RTS2";
    cmdData["Destination"] = destination;
    cmdData["Command"] = command;
    cmdData["Type"] = "Command";
    
    std::string message = SimpleJSON::create(cmdData);
    
    if (!params.empty()) {
        message = message.substr(0, message.length() - 1);
        message += "," + params.substr(1);
    }
    
    return webSocket->sendText(message);
}

void OriginCam::processMessage(const std::string& message)
{
    auto data = SimpleJSON::parse(message);
    
    std::string source = data["Source"];
    std::string command = data["Command"];
    std::string type = data["Type"];
    
    if (type != "Notification" && type != "Response") {
        return;
    }
    
    if (source == "Camera") {
        updateCameraStatus(message);
    }
    else if (source == "ImageServer" && command == "NewImageReady") {
        std::string filePath = data["FileLocation"];
        double ra = std::stod(data.count("Ra") ? data["Ra"] : "0");
        double dec = std::stod(data.count("Dec") ? data["Dec"] : "0");
        handleImageReady(filePath, ra, dec);
    }
}

void OriginCam::updateCameraStatus(const std::string& jsonData)
{
    auto data = SimpleJSON::parse(jsonData);
    
    if (data.find("ISO") != data.end()) {
        cameraStatus->iso = std::stoi(data["ISO"]);
    }
    if (data.find("Exposure") != data.end()) {
        cameraStatus->exposure = std::stod(data["Exposure"]);
    }
    if (data.find("Binning") != data.end()) {
        cameraStatus->binning = std::stoi(data["Binning"]);
    }
    
    cameraStatus->lastUpdate = time(nullptr);
}

void OriginCam::handleImageReady(const std::string& filePath, double ra, double dec)
{
    pendingImagePath = filePath;
    pendingImageRA = ra;
    pendingImageDec = dec;
    
    logStream(MESSAGE_INFO) << "Image ready: " << filePath << sendLog;
    
    // Download and process the image
    if (downloadAndProcessImage(filePath)) {
        imageReady = true;
    } else {
        logStream(MESSAGE_ERROR) << "Failed to download image" << sendLog;
        exposureInProgress = false;
    }
}

bool OriginCam::downloadAndProcessImage(const std::string& filePath)
{
    // Construct URL
    std::string url = "http://" + telescopeHost + filePath;
    
    // Use CURL to download
    CURL *curl = curl_easy_init();
    if (!curl) {
        return false;
    }
    
    std::string imageData;
    
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&imageData);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
    
    CURLcode res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    
    if (res != CURLE_OK) {
        logStream(MESSAGE_ERROR) << "CURL error: " << curl_easy_strerror(res) << sendLog;
        return false;
    }
    
    // Process FITS or TIFF image
    // For now, assume it's FITS and copy the data
    // In reality, you'd need to parse the FITS file properly
    
    if (imageBufferSize < imageData.size()) {
        if (imageBuffer) {
            delete[] imageBuffer;
        }
        imageBuffer = new unsigned char[imageData.size()];
        imageBufferSize = imageData.size();
    }
    
    memcpy(imageBuffer, imageData.data(), imageData.size());
    
    logStream(MESSAGE_INFO) << "Downloaded " << imageData.size() << " bytes" << sendLog;
    
    return true;
}

// Main entry point
int main(int argc, char **argv)
{
    OriginCam device(argc, argv);
    return device.run();
}
