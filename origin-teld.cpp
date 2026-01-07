/*
 * Celestron Origin Telescope Driver for RTS2 - Connect-on-Demand Version
 *
 * Key changes:
 *  - No persistent WebSocket connection
 *  - Connects only when operations are needed
 *  - Maintains connection during active operations (slewing, tracking updates)
 *  - Disconnects when idle
 *  - Uses operation state machine to manage connection lifecycle
 */

#include "origin-teld.h"
#include "origin-websocket.h"
#include "origin-data.h"
#include <libnova/precession.h>
#include <libnova/transform.h>
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

#include <string>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>

static inline double rad2deg(double r) { return r * 180.0 / M_PI; }
static inline double deg2rad(double d) { return d * M_PI / 180.0; }

static inline bool finite2(double a, double b) { return std::isfinite(a) && std::isfinite(b); }

// Simple JSON parser
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
    gotoInProgress = false;

    discoverySocket = -1;
    discovering = false;
    discoveryStartTime = 0;

    operationActive = false;
    lastStatusUpdate = 0;

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

bool Origin::needInfo()
{
    return true;
}

bool Origin::isSafe()
{
    return true;
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
    if (telescopeHost.empty()) {
        useDiscovery = true;
    }

    if (useDiscovery) {
        logStream(MESSAGE_INFO) << "Starting telescope discovery..." << sendLog;

        if (!discoverTelescope()) {
            logStream(MESSAGE_ERROR) << "Failed to discover telescope." << sendLog;
            return -1;
        }

        logStream(MESSAGE_INFO) << "Discovered telescope at " << telescopeHost << sendLog;
        telescopeAddress->setValueCharArr(telescopeHost.c_str());
    }

    if (telescopeHost.empty()) {
        logStream(MESSAGE_ERROR) << "Telescope IP address not specified." << sendLog;
        return -1;
    }

    // Don't connect here - connect on demand
    logStream(MESSAGE_INFO) << "Origin telescope initialized (connect-on-demand mode)" << sendLog;

    // Set initial dummy position
    status->altitude = deg2rad(45.0);
    status->azimuth = deg2rad(180.0);

    return 0;
}

int Origin::info()
{
    time_t now = time(nullptr);

    // If we have an active operation or recent status, stay connected
    if (operationActive || (now - lastStatusUpdate < 5)) {
        ensureConnected();
        pollMessages();
    } else {
        // Idle - disconnect to free WebSocket for camera
        if (connected) {
            logStream(MESSAGE_DEBUG) << "Idle timeout - disconnecting" << sendLog;
            disconnectFromTelescope();
        }
    }

    if (!siteLocationSet)
        return 0;

    if (status->lastUpdate > 0) {
        isAligned->setValueBool(status->isAligned);
        trackingEnabled->setValueBool(status->isTracking);
        batteryVoltage->setValueDouble(status->batteryVoltage);
    }

    return Telescope::info();
}

void Origin::valueChanged(rts2core::Value *changed_value)
{
    Telescope::valueChanged(changed_value);

    if (changed_value == telTargetRaDec)
    {
        double ra  = telTargetRaDec->getRa();
        double dec = telTargetRaDec->getDec();

        logStream(MESSAGE_INFO)
            << "tel_target updated: RA=" << ra << " DEC=" << dec
            << sendLog;

        double ra_rad  = deg2rad(ra);
        double dec_rad = deg2rad(dec);

        std::ostringstream params;
        params << "{"
               << "\"Ra\":"  << ra_rad << ","
               << "\"Dec\":" << dec_rad
               << "}";

        ensureConnected();
        sendCommand("SyncToRaDec", "Mount", params.str());
    }
}

int Origin::startResync()
{
    double targetRa = getTargetRa();    // hours
    double targetDec = getTargetDec();  // degrees

    logStream(MESSAGE_INFO) << "startResync: RA=" << targetRa
                           << " Dec=" << targetDec << sendLog;

    // Ensure we're connected for the goto operation
    if (!ensureConnected()) {
        logStream(MESSAGE_ERROR) << "Failed to connect for goto" << sendLog;
        return -1;
    }

    std::ostringstream params;
    params << "{";
    params << "\"Ra\":" << (targetRa * 15.0 * M_PI / 180.0) << ",";
    params << "\"Dec\":" << (targetDec * M_PI / 180.0);
    params << "}";

    if (!sendCommand("GotoRaDec", "Mount", params.str())) {
        return -1;
    }

    gotoInProgress = true;
    operationActive = true;
    lastStatusUpdate = time(nullptr);

    return 0;
}

int Origin::isMoving()
{
    if (!gotoInProgress) {
        return -2;
    }

    // Keep connection alive and poll status
    ensureConnected();
    pollMessages();
    lastStatusUpdate = time(nullptr);

    if (status->isSlewing) {
        return USEC_SEC / 10;
    }

    // Slewing finished
    gotoInProgress = false;
    operationActive = false;

    logStream(MESSAGE_INFO) << "Goto complete" << sendLog;

    return -2;
}

int Origin::stopMove()
{
    ensureConnected();

    if (!sendCommand("Abort", "Mount")) {
        return -1;
    }

    gotoInProgress = false;
    operationActive = false;

    return 0;
}

int Origin::startPark()
{
    ensureConnected();

    if (!sendCommand("Park", "Mount")) {
        return -1;
    }

    operationActive = true;
    lastStatusUpdate = time(nullptr);

    return 0;
}

int Origin::endPark()
{
    operationActive = false;
    return 0;
}

int Origin::isParking()
{
    ensureConnected();
    pollMessages();
    lastStatusUpdate = time(nullptr);

    if (status->isParked) {
        operationActive = false;
        return -2;
    }

    if (status->isSlewing) {
        return USEC_SEC / 10;
    }

    operationActive = false;
    return -2;
}

int Origin::setTo(double set_ra, double set_dec)
{
    double ra_jnow, dec_jnow;
    j2000ToJNow(set_ra, set_dec, &ra_jnow, &dec_jnow);

    ensureConnected();

    std::ostringstream params;
    params << "{";
    params << "\"Ra\":" << (ra_jnow * 15.0 * M_PI / 180.0) << ",";
    params << "\"Dec\":" << (dec_jnow * M_PI / 180.0);
    params << "}";

    if (!sendCommand("SyncToRaDec", "Mount", params.str())) {
        return -1;
    }

    lastStatusUpdate = time(nullptr);

    return 0;
}

int Origin::correct(double cor_ra, double cor_dec, double real_ra, double real_dec)
{
    (void)cor_ra;
    (void)cor_dec;

    return setTo(real_ra, real_dec);
}

// Private methods

bool Origin::ensureConnected()
{
    if (connected && webSocket && webSocket->isConnected()) {
        return true;
    }

    return connectToTelescope();
}

bool Origin::connectToTelescope()
{
    if (webSocket) {
        delete webSocket;
        webSocket = nullptr;
    }

    webSocket = new OriginWebSocket();

    logStream(MESSAGE_INFO) << "Connecting to telescope at "
                           << telescopeHost << ":" << telescopePort << sendLog;

    if (!webSocket->connect(telescopeHost, telescopePort, "/SmartScope-1.0/mountControlEndpoint")) {
        logStream(MESSAGE_ERROR) << "WebSocket connection failed" << sendLog;
        delete webSocket;
        webSocket = nullptr;
        return false;
    }

    connected = true;

    // Request initial status
    sendCommand("GetStatus", "Mount");

    logStream(MESSAGE_INFO) << "Connected to telescope" << sendLog;

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

    logStream(MESSAGE_DEBUG) << "Disconnected from telescope" << sendLog;
}

void Origin::pollMessages()
{
    if (!webSocket || !webSocket->isConnected()) {
        return;
    }

    int msgCount = 0;
    while (webSocket->hasData() && msgCount < 100) {
        std::string msg = webSocket->receiveText();
        if (!msg.empty()) {
            processMessage(msg);
            msgCount++;
        }
    }

    // Request status update periodically
    time_t now = time(nullptr);
    if (now - lastStatusUpdate >= 1) {
        sendCommand("GetStatus", "Mount");
        lastStatusUpdate = now;
    }
}

bool Origin::sendCommand(const std::string& command,
                         const std::string& destination,
                         const std::string& params)
{
    if (!webSocket || !webSocket->isConnected()) {
        logStream(MESSAGE_ERROR) << "sendCommand: not connected" << sendLog;
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

    return webSocket->sendText(message);
}

void Origin::processMessage(const std::string& message)
{
    auto data = SimpleJSON::parse(message);

    const std::string source  = (data.count("Source")  ? data["Source"]  : "");
    const std::string command = (data.count("Command") ? data["Command"] : "");
    const std::string type    = (data.count("Type")    ? data["Type"]    : "");

    if (source == "Mount")
    {
        updateTelescopeStatus(message);
        return;
    }
}

void Origin::updateTelescopeStatus(const std::string& jsonData)
{
    auto data = SimpleJSON::parse(jsonData);

    auto getD = [&](const char* k, double& out) -> bool {
        auto it = data.find(k);
        if (it == data.end()) return false;
        try {
            out = std::stod(it->second);
            return std::isfinite(out);
        } catch (...) { return false; }
    };

    double altRad = NAN, azmRad = NAN;
    bool haveAlt = getD("Alt", altRad);
    bool haveAzm = getD("Azm", azmRad);
    if (haveAlt && haveAzm) {
        status->altitude = altRad;
        status->azimuth = azmRad;
    }

    double raRad = NAN, decRad = NAN;
    bool haveRa = getD("Ra", raRad);
    bool haveDec = getD("Dec", decRad);
    if (haveRa && haveDec) {
        status->raPosition = raRad;
        status->decPosition = decRad;
        double ra_deg = rad2deg(raRad);
        double dec_deg = rad2deg(decRad);
        setTelRaDec(ra_deg / 15.0, dec_deg);
    }

    double latRad, lonRad;
    bool haveLat = getD("Latitude", latRad);
    bool haveLon = getD("Longitude", lonRad);

    if (haveLat && haveLon && !siteLocationSet) {
        double latDeg = rad2deg(latRad);
        double lonDeg = rad2deg(lonRad);

        setTelLongLat(lonDeg, latDeg);
        setTelAltitude(50.0);

        siteLocationSet = true;

        logStream(MESSAGE_INFO)
            << "Site location set: lat=" << latDeg
            << " lon=" << lonDeg
            << sendLog;
    }

    if (data.find("IsAligned") != data.end())
        status->isAligned = (data["IsAligned"] == "true");

    if (data.find("IsTracking") != data.end())
        status->isTracking = (data["IsTracking"] == "true");

    if (data.find("IsGotoOver") != data.end())
        status->isSlewing = (data["IsGotoOver"] == "false");

    if (data.find("BatteryVoltage") != data.end()) {
        try { status->batteryVoltage = std::stod(data["BatteryVoltage"]); }
        catch (...) {}
    }

    status->lastUpdate = time(nullptr);
}

void Origin::j2000ToJNow(double ra_j2000, double dec_j2000, double *ra_jnow, double *dec_jnow)
{
    double jd = ln_get_julian_from_sys();

    ln_equ_posn pos_j2000, pos_jnow;
    pos_j2000.ra = ra_j2000 * 15.0;
    pos_j2000.dec = dec_j2000;

    ln_get_equ_prec2(&pos_j2000, 2451545.0, jd, &pos_jnow);

    *ra_jnow = pos_jnow.ra / 15.0;
    *dec_jnow = pos_jnow.dec;
}

void Origin::jnowToJ2000(double ra_jnow, double dec_jnow, double *ra_j2000, double *dec_j2000)
{
    double jd = ln_get_julian_from_sys();

    ln_equ_posn pos_jnow, pos_j2000;
    pos_jnow.ra = ra_jnow * 15.0;
    pos_jnow.dec = dec_jnow;

    ln_get_equ_prec2(&pos_jnow, jd, 2451545.0, &pos_j2000);

    *ra_j2000 = pos_j2000.ra / 15.0;
    *dec_j2000 = pos_j2000.dec;
}

// Discovery implementation (unchanged from original)

bool Origin::discoverTelescope()
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

bool Origin::startDiscovery()
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

int main(int argc, char **argv)
{
    Origin device(argc, argv);
    return device.run();
}
