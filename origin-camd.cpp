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
#include <fitsio.h>

// Network headers for discovery
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <algorithm>

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
    imageReady = false;
    
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
    gain->setValueInteger(800);
    
    createValue(previewMode, "PREVIEW", "preview mode (faster downloads)", true, RTS2_VALUE_WRITABLE);
    previewMode->setValueBool(false);
    
    createValue(ccdTemp, "CCD_TEMP", "CCD temperature", false);
    ccdTemp->setValueDouble(20);
    
    addOption('a', "address", 1, "telescope IP address (optional if using discovery)");
    addOption('p', "port", 1, "telescope port (default: 80)");
    addOption('D', "discover", 0, "auto-discover telescope on network");
}

OriginCam::~OriginCam()
{
    stopDiscovery();
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
            
        case 'D':
            useDiscovery = true;
            return 0;
            
        default:
            return Camera::processOption(opt);
    }
}

int OriginCam::initHardware()
{
    // If no address specified, use discovery
    if (telescopeHost.empty()) {
        useDiscovery = true;
    }
    
    if (useDiscovery) {
        logStream(MESSAGE_INFO) << "Starting telescope discovery..." << sendLog;
        
        if (!discoverTelescope()) {
            logStream(MESSAGE_ERROR) << "Failed to discover telescope. You can specify IP with -a option." << sendLog;
            return -1;
        }
        
        logStream(MESSAGE_INFO) << "Discovered telescope at " << telescopeHost << sendLog;
        telescopeAddress->setValueCharArr(telescopeHost.c_str());
    }
    
    if (telescopeHost.empty()) {
        logStream(MESSAGE_ERROR) << "Telescope IP address not specified and discovery failed. Use -a option or -D for discovery." << sendLog;
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
    int width = 3056;
    int height = 2048;
    float pixelSize = 3.76;
    
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
    exposureInProgress = false;
    imageReady = false;

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
    
    if (!webSocket->connect(telescopeHost, telescopePort, "/SmartScope-1.0/mountControlEndpoint")) {
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
    pendingImageRA   = ra;
    pendingImageDec  = dec;

    logStream(MESSAGE_INFO) << "Image ready: " << filePath << sendLog;

    std::string url =
        "http://" + telescopeHost + "/SmartScope-1.0/dev2/" + filePath;

    std::vector<uint8_t> imageData = downloadImageSync(url);

    if (imageData.empty())
    {
        logStream(MESSAGE_ERROR) << "Image download failed" << sendLog;
        exposureInProgress = false;
        return;
    }

    if (decodeTIFF(imageData))
    {
        imageReady = true;
    }
    else
    {
        logStream(MESSAGE_ERROR) << "TIFF decode failed" << sendLog;
        exposureInProgress = false;
    }
}

bool OriginCam::decodeTIFF(const std::vector<uint8_t> &imageData)
{
    // Write to temp file (libtiff needs a filename)
    std::string tmp = "/tmp/origin_image.tiff";
    std::ofstream f(tmp, std::ios::binary);
    f.write(reinterpret_cast<const char*>(imageData.data()), imageData.size());
    f.close();

    TIFF *tif = TIFFOpen(tmp.c_str(), "r");
    if (!tif)
        return false;

    uint32_t width, height;
    uint16_t spp, bps, config;

    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
    TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &spp);
    TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bps);
    TIFFGetField(tif, TIFFTAG_PLANARCONFIG, &config);

    if (spp != 3 || bps != 16)
    {
        TIFFClose(tif);
        return false;
    }

    size_t planeSize = width * height;

    uint16_t *dst = reinterpret_cast<uint16_t *>(getDataBuffer(0));

    std::vector<uint16_t> scanline(width * 3);

    for (uint32 row = 0; row < height; row++)
    {
        TIFFReadScanline(tif, scanline.data(), row);

        for (uint32 col = 0; col < width; col++)
        {
            size_t i = row * width + col;
            size_t s = col * 3;

            dst[i]                 = scanline[s + 0]; // R
            dst[planeSize + i]     = scanline[s + 1]; // G
            dst[planeSize*2 + i]   = scanline[s + 2]; // B
        }
    }

    TIFFClose(tif);
    return true;
}

// Discovery implementation (same as telescope driver)

bool OriginCam::discoverTelescope()
{
    if (!startDiscovery()) {
        return false;
    }
    
    // Poll for up to 30 seconds
    time_t startTime = time(nullptr);
    while (time(nullptr) - startTime < 30) {
        pollDiscovery();
        
        if (!telescopeHost.empty()) {
            stopDiscovery();
            return true;
        }
        
        usleep(100000);  // 100ms
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
        std::string datagram(buffer, bytesRead);
        
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

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <iostream>

std::vector<uint8_t>
OriginCam::downloadImageSync(const std::string &url)
{
    std::vector<uint8_t> result;

    // ---- Parse URL (very simple HTTP only) ----
    // expected: http://host[:port]/path
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

    std::cerr << "Downloading from " << host
              << ":" << port
              << " path=" << path << std::endl;

    // ---- Create socket ----
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        perror("socket");
        return result;
    }

    // ---- Timeouts ----
    struct timeval timeout;
    timeout.tv_sec = 60;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    // ---- Resolve host ----
    struct hostent *server = gethostbyname(host.c_str());
    if (!server)
    {
        std::cerr << "gethostbyname failed for " << host << std::endl;
        close(sock);
        return result;
    }

    // ---- Connect ----
    struct sockaddr_in serv_addr {};
    serv_addr.sin_family = AF_INET;
    std::memcpy(&serv_addr.sin_addr.s_addr,
                server->h_addr,
                server->h_length);
    serv_addr.sin_port = htons(port);

    if (connect(sock,
                reinterpret_cast<struct sockaddr *>(&serv_addr),
                sizeof(serv_addr)) < 0)
    {
        perror("connect");
        close(sock);
        return result;
    }

    // ---- Send GET ----
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

    // ---- Read response ----
    std::vector<uint8_t> response;
    response.reserve(8 * 1024 * 1024);

    char buffer[65536];
    ssize_t n;
    size_t totalBytes = 0;

    auto lastKeepalive = std::chrono::steady_clock::now();
    auto lastLog = lastKeepalive;

    while (true)
    {
        n = recv(sock, buffer, sizeof(buffer), 0);
        if (n > 0)
        {
            response.insert(response.end(), buffer, buffer + n);
            totalBytes += n;
        }
        else if (n == 0)
        {
            break; // connection closed
        }
        else if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            perror("recv");
            break;
        }

        auto now = std::chrono::steady_clock::now();

        // ---- WebSocket keepalive every 5s ----
        if (std::chrono::duration_cast<std::chrono::seconds>(
                now - lastKeepalive)
                .count() >= 5)
        {
            lastKeepalive = now;
        }

        // ---- Progress log ----
        if (std::chrono::duration_cast<std::chrono::seconds>(
                now - lastLog)
                .count() >= 5)
        {
            std::cerr << "Downloaded " << totalBytes << " bytes...\n";
            lastLog = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    close(sock);

    std::cerr << "Total received: " << response.size() << " bytes\n";

    // ---- Strip HTTP headers ----
    const std::string headerSep = "\r\n\r\n";
    auto it = std::search(response.begin(), response.end(),
                          headerSep.begin(), headerSep.end());

    if (it == response.end())
    {
        std::cerr << "HTTP header not found\n";
        return result;
    }

    it += headerSep.size();
    result.assign(it, response.end());

    std::cerr << "Image payload size: " << result.size() << " bytes\n";

    return result;
}

// Main entry point
int main(int argc, char **argv)
{
    OriginCam device(argc, argv);
    return device.run();
}
