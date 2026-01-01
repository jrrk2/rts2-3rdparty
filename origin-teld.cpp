/*
 * Celestron Origin Telescope Driver for RTS2
 */

#include "origin-teld.h"
#include "origin-websocket.h"
#include "origin-data.h"
#include <libnova/precession.h>
#include <libnova/julian_day.h>
#include <sstream>
#include <cstring>
#include <cmath>
#include <unistd.h>

// Network headers for discovery
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <algorithm>

// JSON parsing helper (simple implementation)
#include <string>
#include <map>

// Simple JSON parser for our needs
class SimpleJSON {
public:
    static std::map<std::string, std::string> parse(const std::string& json) {
        std::map<std::string, std::string> result;
        // Very basic parser - extract key:value pairs
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
                    // Trim whitespace
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
            // Check if value is numeric
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

using namespace rts2teld;

Origin::Origin(int argc, char **argv)
    : Telescope(argc, argv)
{
    telescopeHost = "";
    telescopePort = 80;
    useDiscovery = false;
    webSocket = nullptr;
    connected = false;
    status = new TelescopeStatus();
    nextSequenceId = 1;
    targetRA = 0;
    targetDec = 0;
    gotoInProgress = false;
    
    discoverySocket = -1;
    discovering = false;
    discoveryStartTime = 0;
    
    createValue(telescopeAddress, "telescope_address", "telescope IP address", false);
    telescopeAddress->setValueCharArr("");
    
    createValue(isAligned, "is_aligned", "mount alignment status", false);
    isAligned->setValueBool(false);
    
    createValue(trackingEnabled, "tracking", "tracking enabled", true, RTS2_VALUE_WRITABLE);
    trackingEnabled->setValueBool(false);
    
    createValue(batteryVoltage, "battery_voltage", "battery voltage", false);
    batteryVoltage->setValueDouble(0);
    
    createValue(temperature, "ccd_temp", "CCD temperature", false);
    temperature->setValueDouble(20);
    
    addOption('a', "address", 1, "telescope IP address (optional if using discovery)");
    addOption('p', "port", 1, "telescope port (default: 80)");
    addOption('D', "discover", 0, "auto-discover telescope on network");
}

Origin::~Origin()
{
    stopDiscovery();
    disconnectFromTelescope();
    delete status;
    if (webSocket) {
        delete webSocket;
    }
}

int Origin::processOption(int opt)
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
            return Telescope::processOption(opt);
    }
}

int Origin::initHardware()
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
    
    // Set initial position (will be updated by status messages)
    setTelRa(0);
    setTelDec(0);
    
    logStream(MESSAGE_INFO) << "Origin telescope initialized successfully" << sendLog;
    
    return 0;
}

int Origin::info()
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
    
    // Update RTS2 with current position
    if (status->lastUpdate > 0) {
        setTelRa(status->raPosition);
        setTelDec(status->decPosition);
        
        isAligned->setValueBool(status->isAligned);
        trackingEnabled->setValueBool(status->isTracking);
        batteryVoltage->setValueDouble(status->batteryVoltage);
    }
    
    return Telescope::info();
}

int Origin::startResync()
{
    // RTS2's Telescope class provides these public methods to get the target
    // The target is already stored when startResync() is called
    double target_ra = getTargetRa();    // In hours
    double target_dec = getTargetDec();   // In degrees
    
    // Convert J2000 coordinates to JNow for the mount
    double ra_jnow, dec_jnow;
    j2000ToJNow(target_ra, target_dec, &ra_jnow, &dec_jnow);
    
    targetRA = ra_jnow;
    targetDec = dec_jnow;
    
    // Build goto command
    std::ostringstream params;
    params << "{";
    params << "\"Ra\":" << (targetRA * 15.0 * M_PI / 180.0) << ",";  // Convert hours to radians
    params << "\"Dec\":" << (targetDec * M_PI / 180.0);               // Convert degrees to radians
    params << "}";
    
    if (!sendCommand("GotoRaDec", "Mount", params.str())) {
        return -1;
    }
    
    gotoInProgress = true;
    
    return 0;
}

int Origin::isMoving()
{
    if (!gotoInProgress) {
        return -2;  // Not moving
    }
    
    if (status->isSlewing) {
        return USEC_SEC / 10;  // Check again in 100ms
    }
    
    // Slewing finished
    gotoInProgress = false;
    return -2;
}

int Origin::stopMove()
{
    if (!sendCommand("Abort", "Mount")) {
        return -1;
    }
    
    gotoInProgress = false;
    
    return 0;
}

int Origin::startPark()
{
    if (!sendCommand("Park", "Mount")) {
        return -1;
    }
    
    return 0;
}

int Origin::endPark()
{
    return 0;
}

int Origin::isParking()
{
    if (status->isParked) {
        return -2;  // Parked
    }
    
    if (status->isSlewing) {
        return USEC_SEC / 10;  // Still parking
    }
    
    return -2;  // Done parking
}

int Origin::setTo(double set_ra, double set_dec)
{
    // Convert J2000 to JNow
    double ra_jnow, dec_jnow;
    j2000ToJNow(set_ra, set_dec, &ra_jnow, &dec_jnow);
    
    // Build sync command
    std::ostringstream params;
    params << "{";
    params << "\"Ra\":" << (ra_jnow * 15.0 * M_PI / 180.0) << ",";
    params << "\"Dec\":" << (dec_jnow * M_PI / 180.0);
    params << "}";
    
    if (!sendCommand("SyncToRaDec", "Mount", params.str())) {
        return -1;
    }
    
    return 0;
}

int Origin::correct(double cor_ra, double cor_dec, double real_ra, double real_dec)
{
    // Suppress unused parameter warnings
    (void)cor_ra;
    (void)cor_dec;
    
    // Use the sync functionality
    return setTo(real_ra, real_dec);
}

// Private methods

bool Origin::connectToTelescope()
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
    sendCommand("GetMountStatus", "Mount");
    
    logStream(MESSAGE_INFO) << "Connected to telescope via WebSocket" << sendLog;
    
    return true;
}

void Origin::disconnectFromTelescope()
{
    if (webSocket) {
        webSocket->disconnect();
        delete webSocket;
        webSocket = nullptr;
    }
    connected = false;
}

bool Origin::sendCommand(const std::string& command, const std::string& destination,
                         const std::string& params)
{
    if (!webSocket || !webSocket->isConnected()) {
        return false;
    }
    
    std::map<std::string, std::string> cmdData;
    cmdData["Sequence"] = std::to_string(nextSequenceId++);
    cmdData["Source"] = "INDI";
    cmdData["Destination"] = destination;
    cmdData["Command"] = command;
    cmdData["Type"] = "Command";
    
    std::string message = SimpleJSON::create(cmdData);
    
    // If we have params, we need to merge them
    if (!params.empty()) {
        // Remove trailing }
        message = message.substr(0, message.length() - 1);
        // Add params (remove leading {)
        message += "," + params.substr(1);
    }
    
    return webSocket->sendText(message);
}

void Origin::processMessage(const std::string& message)
{
    auto data = SimpleJSON::parse(message);
    
    std::string source = data["Source"];
    std::string command = data["Command"];
    std::string type = data["Type"];
    
    // Only process notifications and responses
    if (type != "Notification" && type != "Response") {
        return;
    }
    
    if (source == "Mount") {
        updateTelescopeStatus(message);
    }
}

void Origin::updateTelescopeStatus(const std::string& jsonData)
{
    auto data = SimpleJSON::parse(jsonData);
    
    // Update status from JSON
    if (data.find("Alt") != data.end()) {
        status->altitude = std::stod(data["Alt"]);
    }
    if (data.find("Azm") != data.end()) {
        status->azimuth = std::stod(data["Azm"]);
    }
    if (data.find("IsAligned") != data.end()) {
        status->isAligned = (data["IsAligned"] == "true");
    }
    if (data.find("IsTracking") != data.end()) {
        status->isTracking = (data["IsTracking"] == "true");
    }
    if (data.find("IsGotoOver") != data.end()) {
        status->isSlewing = (data["IsGotoOver"] == "false");
    }
    if (data.find("BatteryVoltage") != data.end()) {
        status->batteryVoltage = std::stod(data["BatteryVoltage"]);
    }
    
    // Convert Alt/Az to RA/Dec
    // This is a simplified conversion - in reality you'd need proper coordinate transformation
    // For now, we'll use the mount's reported positions
    status->raPosition = getTelRa();
    status->decPosition = getTelDec();
    
    status->lastUpdate = time(nullptr);
}

void Origin::j2000ToJNow(double ra_j2000, double dec_j2000, double *ra_jnow, double *dec_jnow)
{
    double jd = ln_get_julian_from_sys();
    
    ln_equ_posn pos_j2000, pos_jnow;
    pos_j2000.ra = ra_j2000 * 15.0;   // Hours to degrees
    pos_j2000.dec = dec_j2000;
    
    // Precess from J2000 (JD 2451545.0) to current epoch
    ln_get_equ_prec2(&pos_j2000, 2451545.0, jd, &pos_jnow);
    
    *ra_jnow = pos_jnow.ra / 15.0;    // Degrees to hours
    *dec_jnow = pos_jnow.dec;
}

void Origin::jnowToJ2000(double ra_jnow, double dec_jnow, double *ra_j2000, double *dec_j2000)
{
    double jd = ln_get_julian_from_sys();
    
    ln_equ_posn pos_jnow, pos_j2000;
    pos_jnow.ra = ra_jnow * 15.0;     // Hours to degrees
    pos_jnow.dec = dec_jnow;
    
    // Precess from current epoch to J2000 (JD 2451545.0)
    ln_get_equ_prec2(&pos_jnow, jd, 2451545.0, &pos_j2000);
    
    *ra_j2000 = pos_j2000.ra / 15.0;  // Degrees to hours
    *dec_j2000 = pos_j2000.dec;
}

// Discovery implementation

bool Origin::discoverTelescope()
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

bool Origin::startDiscovery()
{
    logStream(MESSAGE_INFO) << "Starting UDP discovery on port 55555..." << sendLog;
    
    // Close existing socket if open
    if (discoverySocket >= 0) {
        close(discoverySocket);
        discoverySocket = -1;
    }
    
    // Create UDP socket
    discoverySocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (discoverySocket < 0) {
        logStream(MESSAGE_ERROR) << "Failed to create UDP socket: " << strerror(errno) << sendLog;
        return false;
    }
    
    // Set socket to non-blocking mode
    int flags = fcntl(discoverySocket, F_GETFL, 0);
    fcntl(discoverySocket, F_SETFL, flags | O_NONBLOCK);
    
    // Enable address reuse
    int reuse = 1;
    setsockopt(discoverySocket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    
#ifdef SO_REUSEPORT
    setsockopt(discoverySocket, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif
    
    // Enable broadcast reception
    int broadcast = 1;
    setsockopt(discoverySocket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
    
    // Bind to port 55555 on all interfaces
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

void Origin::stopDiscovery()
{
    if (discoverySocket >= 0) {
        close(discoverySocket);
        discoverySocket = -1;
    }
    discovering = false;
}

void Origin::pollDiscovery()
{
    if (!discovering || discoverySocket < 0) {
        return;
    }
    
    char buffer[4096];
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);
    
    // Read all available datagrams
    while (discoverySocket >= 0) {
        ssize_t bytesRead = recvfrom(discoverySocket, buffer, sizeof(buffer) - 1, 0,
                                     (struct sockaddr*)&sender_addr, &sender_len);
        
        if (bytesRead < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No more data available
                break;
            } else {
                logStream(MESSAGE_ERROR) << "recvfrom error: " << strerror(errno) << sendLog;
                break;
            }
        }
        
        if (bytesRead == 0) {
            break;
        }
        
        // Null-terminate the data
        buffer[bytesRead] = '\0';
        std::string datagram(buffer, bytesRead);
        
        // Get sender IP
        char sender_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &sender_addr.sin_addr, sender_ip, INET_ADDRSTRLEN);
        
        // Check if this looks like a telescope broadcast
        if (datagram.find("Origin") != std::string::npos && 
            datagram.find("IP Address") != std::string::npos) {
            
            // Extract IP address using regex
            std::string extractedIP;
            size_t pos = 0;
            while ((pos = datagram.find('.', pos)) != std::string::npos) {
                // Try to find an IP pattern
                size_t start = datagram.rfind(' ', pos);
                if (start == std::string::npos) start = 0;
                else start++;
                
                size_t end = datagram.find(' ', pos);
                if (end == std::string::npos) end = datagram.length();
                
                std::string candidate = datagram.substr(start, end - start);
                
                // Simple validation: does it look like an IP?
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

// Main entry point
int main(int argc, char **argv)
{
    Origin device(argc, argv);
    return device.run();
}
