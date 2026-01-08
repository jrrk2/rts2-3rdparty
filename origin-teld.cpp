/*
 * Celestron Origin Telescope Driver for RTS2
 *
 * DIAGNOSTIC BUILD: persistent WebSocket during initialisation + active ops,
 * with controlled idle disconnect, plus robust-ish parsing and raw frame logging.
 *
 * Goal:
 *  - Prove whether Latitude/Longitude ever arrive on the mount websocket
 *  - Prove whether we are disconnecting before they arrive
 *  - Prove whether JSON parsing is failing on "semi-JSON" (single quotes / ';' / hex escapes)
 *
 * How to use:
 *   - Set env ORIGIN_TELD_DEBUG=1 for extra logs and raw frame sampling
 *   - Set env ORIGIN_TELD_RAW=1 to log every Mount frame (can be noisy)
 *
 * Notes:
 *  - This file intentionally keeps parsing permissive to diagnose mixed formats.
 *  - It does NOT attempt to “fix” everything; it instruments what happens.
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
#include <ctime>
#include <cstdlib>

// Network headers for discovery
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <algorithm>

#include <string>
#include <map>
#include <mutex>

using namespace rts2teld;

static inline double rad2deg(double r) { return r * 180.0 / M_PI; }
static inline double deg2rad(double d) { return d * M_PI / 180.0; }
static inline bool isfinite_d(double x) { return std::isfinite(x); }

/* -------------------------
 * Permissive "JSON-ish" parser
 *
 * Handles:
 *  - "key":"value"   (normal JSON)
 *  - 'key':'value'   (single quotes)
 *  - separators: , or ;
 *  - bare values: true/false/null/NaN/-1.23
 *  - hex escapes: \xNN
 *  - unicode escapes: \uXXXX
 *
 * It is deliberately shallow: key/value extraction only.
 * ------------------------- */
class SimpleJSON {
public:
    static bool isNullLike(const std::string& v)
    {
        if (v.empty()) return true;
        if (v == "null" || v == "NULL") return true;
        if (v == "NaN"  || v == "nan"  || v == "NAN") return true;
        return false;
    }

    static std::string unescape(const std::string& s)
    {
        std::string out;
        out.reserve(s.size());

        auto hexval = [](char c)->int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
            if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
            return -1;
        };

        for (size_t i = 0; i < s.size(); i++) {
            char c = s[i];
            if (c != '\\' || i + 1 >= s.size()) {
                out.push_back(c);
                continue;
            }
            char n = s[i + 1];
            if (n == 'n') { out.push_back('\n'); i++; continue; }
            if (n == 'r') { out.push_back('\r'); i++; continue; }
            if (n == 't') { out.push_back('\t'); i++; continue; }
            if (n == '\\') { out.push_back('\\'); i++; continue; }
            if (n == '"') { out.push_back('"'); i++; continue; }
            if (n == '\'') { out.push_back('\''); i++; continue; }

            // \xNN
            if (n == 'x' && i + 3 < s.size()) {
                int h1 = hexval(s[i + 2]);
                int h2 = hexval(s[i + 3]);
                if (h1 >= 0 && h2 >= 0) {
                    out.push_back(static_cast<char>((h1 << 4) | h2));
                    i += 3;
                    continue;
                }
            }

            // \uXXXX (very minimal: decode BMP to UTF-8)
            if (n == 'u' && i + 5 < s.size()) {
                int h[4];
                h[0] = hexval(s[i + 2]);
                h[1] = hexval(s[i + 3]);
                h[2] = hexval(s[i + 4]);
                h[3] = hexval(s[i + 5]);
                if (h[0] >= 0 && h[1] >= 0 && h[2] >= 0 && h[3] >= 0) {
                    int code = (h[0] << 12) | (h[1] << 8) | (h[2] << 4) | h[3];
                    if (code < 0x80) {
                        out.push_back(static_cast<char>(code));
                    } else if (code < 0x800) {
                        out.push_back(static_cast<char>(0xC0 | ((code >> 6) & 0x1F)));
                        out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
                    } else {
                        out.push_back(static_cast<char>(0xE0 | ((code >> 12) & 0x0F)));
                        out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
                        out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
                    }
                    i += 5;
                    continue;
                }
            }

            // fallback: keep escaped char
            out.push_back(n);
            i++;
        }
        return out;
    }

    static std::map<std::string, std::string> parse(const std::string& input)
    {
        std::string json = input;
        // Common “semi-json” from some captures: leading/trailing whitespace
        trimInPlace(json);

        std::map<std::string, std::string> result;
        size_t pos = 0;

        auto skipWS = [&](size_t& p) {
            while (p < json.size() && (json[p] == ' ' || json[p] == '\t' || json[p] == '\r' || json[p] == '\n'))
                p++;
        };

        auto parseQuoted = [&](size_t& p)->std::string {
            // expects " or '
            char q = json[p];
            p++; // consume quote
            std::string v;
            while (p < json.size()) {
                char c = json[p++];
                if (c == q) break;
                if (c == '\\' && p < json.size()) {
                    v.push_back('\\');
                    v.push_back(json[p++]); // keep escapes for later unescape()
                } else {
                    v.push_back(c);
                }
            }
            return unescape(v);
        };

        auto parseBare = [&](size_t& p)->std::string {
            size_t start = p;
            while (p < json.size()) {
                char c = json[p];
                // stop at separators or end object
                if (c == ',' || c == ';' || c == '}' || c == '\n' || c == '\r')
                    break;
                p++;
            }
            std::string v = json.substr(start, p - start);
            trimInPlace(v);
            return v;
        };

        while (pos < json.size()) {
            // Find key start: quote or bare token before :
            skipWS(pos);

            // Skip punctuation until plausible key
            while (pos < json.size() && json[pos] != '"' && json[pos] != '\'' && !isalnum(static_cast<unsigned char>(json[pos]))) {
                pos++;
                skipWS(pos);
            }
            if (pos >= json.size()) break;

            std::string key;
            if (json[pos] == '"' || json[pos] == '\'') {
                key = parseQuoted(pos);
            } else {
                // bare key until ':' or whitespace
                size_t ks = pos;
                while (pos < json.size()) {
                    char c = json[pos];
                    if (c == ':' || c == '=' || c == ' ' || c == '\t' || c == '\r' || c == '\n') break;
                    pos++;
                }
                key = json.substr(ks, pos - ks);
                trimInPlace(key);
            }

            // Find colon (or '=')
            while (pos < json.size() && json[pos] != ':' && json[pos] != '=') pos++;
            if (pos >= json.size()) break;
            pos++; // consume ':' or '='
            skipWS(pos);

            // Parse value
            std::string value;
            if (pos < json.size() && (json[pos] == '"' || json[pos] == '\'')) {
                value = parseQuoted(pos);
            } else {
                value = parseBare(pos);
            }

            if (!key.empty())
                result[key] = value;

            // Skip to next
            while (pos < json.size() && json[pos] != ',' && json[pos] != ';' && json[pos] != '}' ) pos++;
            if (pos < json.size() && (json[pos] == ',' || json[pos] == ';')) pos++;
        }

        return result;
    }

    static std::string create(const std::map<std::string, std::string>& data)
    {
        std::ostringstream json;
        json << "{";
        bool first = true;
        for (const auto& pair : data) {
            if (!first) json << ",";
            json << "\"" << pair.first << "\":";
            bool isNumeric = true;
            for (char c : pair.second) {
                if (!isdigit(static_cast<unsigned char>(c)) && c != '.' && c != '-' && c != 'e' && c != 'E') {
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

private:
    static void trimInPlace(std::string& s)
    {
        size_t a = s.find_first_not_of(" \t\r\n");
        if (a == std::string::npos) { s.clear(); return; }
        size_t b = s.find_last_not_of(" \t\r\n");
        s = s.substr(a, (b - a) + 1);
    }
};

/* -------------------------
 * Origin driver
 * ------------------------- */

Origin::Origin(int argc, char** argv)
    : Telescope(argc, argv,
                true,   // diffTrack
                true,   // hasTracking (needed for ignoreHorizon paths)
                0,      // hasUnTelCoordinates (we are NOT an alt/az driver)
                false,  // hasAltAzDiff
                false,  // parkingBlock
                false)  // hasDerotators
{
    telescopeHost = "";
    telescopePort = 80;
    useDiscovery = false;

    webSocket = nullptr;
    connected = false;
    status = new TelescopeStatus();
    nextSequenceId = 1;

    discoverySocket = -1;
    discovering = false;
    discoveryStartTime = 0;

    gotoInProgress = false;
    operationActive = false;

    // Lifecycle flags
    initialising = true;            // stay connected until we have site
    siteLocationSet = false;
    lastRxTime = 0;
    lastTxTime = 0;
    lastStatusUpdate = 0;
    lastActivityTime = 0;

    debugEnabled = (getenv("ORIGIN_TELD_DEBUG") != nullptr);
    rawFrameLogging = (getenv("ORIGIN_TELD_RAW") != nullptr);

    createValue(telescopeAddress, "telescope_address", "telescope IP address", false);
    telescopeAddress->setValueCharArr("");

    createValue(isAligned, "is_aligned", "mount alignment status", false);
    isAligned->setValueBool(false);

    createValue(batteryVoltage, "battery_voltage", "battery voltage", false);
    batteryVoltage->setValueDouble(0);

    createValue(temperature, "ccd_temp", "CCD temperature", false);
    temperature->setValueDouble(20);

    addOption('a', "address", 1, "telescope IP address (optional if using discovery)");
    addOption('p', "port", 1, "telescope port (default: 80)");
    addOption('D', "discover", 0, "auto-discover telescope on network");

    setDeviceName("ORIGIN_CAM");
}

Origin::~Origin()
{
    stopDiscovery();
    disconnectFromTelescope();
    delete status;
    status = nullptr;
    if (webSocket) {
        delete webSocket;
        webSocket = nullptr;
    }
}

bool Origin::needInfo() { return true; }
bool Origin::isSafe() { return true; }

int Origin::processOption(int opt)
{
    switch (opt) {
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
    logStream(MESSAGE_INFO) << "=== Origin Telescope Driver Starting (DIAG) ===" << sendLog;
    logStream(MESSAGE_INFO) << "Mode: persistent-until-initialised, then idle-disconnect" << sendLog;
    if (debugEnabled) logStream(MESSAGE_INFO) << "Debug: ORIGIN_TELD_DEBUG=1" << sendLog;
    if (rawFrameLogging) logStream(MESSAGE_WARNING) << "Raw frame logging enabled: ORIGIN_TELD_RAW=1 (noisy)" << sendLog;

    if (telescopeHost.empty()) {
        useDiscovery = true;
        logStream(MESSAGE_INFO) << "No host specified, will use discovery" << sendLog;
    } else {
        logStream(MESSAGE_INFO) << "Using specified host: " << telescopeHost << sendLog;
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

    // We intentionally connect immediately during init, and hold until site is known.
    initialising = true;
    operationActive = false;
    lastActivityTime = time(nullptr);

    if (!ensureConnected()) {
        logStream(MESSAGE_ERROR) << "Initial connect failed" << sendLog;
        return -1;
    }

    logStream(MESSAGE_INFO) << "Connected; awaiting site (Latitude/Longitude) from Mount frames" << sendLog;

    return 0;
}

int Origin::initValues()
{
    int ret = Telescope::initValues();
    if (ret)
        return ret;

    rts2core::Value *v = getValue(getDeviceName(), "IGNORE_HORIZON");
    if (v)
    {
        auto *vb = dynamic_cast<rts2core::ValueBool *>(v);
        if (vb)
        {
            vb->setValueBool(true);
            sendValueAll(vb);
            logStream(MESSAGE_INFO) << "IGNORE_HORIZON enabled (Origin uses RA/DEC only)";
        }
    }

    return 0;
}

void Origin::valueChanged(rts2core::Value* changed_value)
{
    Telescope::valueChanged(changed_value);

    if (changed_value == telTargetRaDec) {
        double ra_deg = telTargetRaDec->getRa();   // degrees in RTS2 target value
        double dec_deg = telTargetRaDec->getDec(); // degrees

        logStream(MESSAGE_INFO) << "tel_target updated: RA=" << ra_deg << "deg DEC=" << dec_deg << "deg" << sendLog;

        // Origin expects radians (based on your previous working)
        double ra_rad = deg2rad(ra_deg);
        double dec_rad = deg2rad(dec_deg);

        std::ostringstream params;
        params << "{"
               << "\"Ra\":" << ra_rad << ","
               << "\"Dec\":" << dec_rad
               << "}";

        operationActive = true;
        lastActivityTime = time(nullptr);

        ensureConnected();
        sendCommand("GotoRaDec", "Mount", params.str());
    }
}

int Origin::idle()
{
	updateConnectionPolicy();
	return Telescope::idle();
}

int Origin::info()
{
    if (connected) {
        pollMessages(); // drains inbound; also triggers periodic GetStatus
    }

    // Only allow Telescope::info once we have a site. (This mirrors what you wanted.)
    if (!siteLocationSet)
        return 0;

    if (status->lastUpdate > 0) {
        isAligned->setValueBool(status->isAligned);
        batteryVoltage->setValueDouble(status->batteryVoltage);
    }

    return Telescope::info();
}

int Origin::startResync()
{
    double targetRaHours = getTargetRa();    // hours
    double targetDecDeg  = getTargetDec();   // degrees

    logStream(MESSAGE_INFO) << "startResync: target RA=" << targetRaHours << "h DEC=" << targetDecDeg << "deg" << sendLog;

    double ra_rad  = targetRaHours * 15.0 * M_PI / 180.0;
    double dec_rad = targetDecDeg * M_PI / 180.0;

    std::ostringstream params;
    params << "{"
           << "\"Ra\":"  << ra_rad << ","
           << "\"Dec\":" << dec_rad
           << "}";

    operationActive = true;
    status->isSlewing = true;
    gotoInProgress = true;
    lastActivityTime = time(nullptr);

    if (!ensureConnected()) {
        logStream(MESSAGE_ERROR) << "startResync: failed to connect" << sendLog;
        return -1;
    }

    logStream(MESSAGE_INFO) << "startResync: sending GotoRaDec " << params.str() << sendLog;

    if (!sendCommand("GotoRaDec", "Mount", params.str())) {
        logStream(MESSAGE_ERROR) << "startResync: sendCommand(GotoRaDec) failed" << sendLog;
        gotoInProgress = false;
        operationActive = false;
        return -1;
    }

    return 0;
}

int Origin::isMoving()
{
    ensureConnected();
    pollMessages();

    if (status->isSlewing) {
        return USEC_SEC / 10;
    }

    if (gotoInProgress) {
        gotoInProgress = false;
        operationActive = false;
        logStream(MESSAGE_INFO) << "Goto complete" << sendLog;
    }

    return -2;
}

int Origin::stopMove()
{
    ensureConnected();
    operationActive = true;
    lastActivityTime = time(nullptr);

    if (!sendCommand("AbortAxisMovement", "Mount")) {
        logStream(MESSAGE_ERROR) << "stopMove: AbortAxisMovement failed" << sendLog;
        return -1;
    }

    gotoInProgress = false;
    operationActive = false;
    return 0;
}

int Origin::startPark()
{
    ensureConnected();
    operationActive = true;
    lastActivityTime = time(nullptr);

    if (!sendCommand("Park", "Mount")) {
        logStream(MESSAGE_ERROR) << "startPark: Park failed" << sendLog;
        operationActive = false;
        return -1;
    }
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
    // J2000 -> JNow
    double ra_jnow, dec_jnow;
    j2000ToJNow(set_ra, set_dec, &ra_jnow, &dec_jnow);

    ensureConnected();
    operationActive = true;
    lastActivityTime = time(nullptr);

    std::ostringstream params;
    params << "{"
           << "\"Ra\":"  << (ra_jnow * 15.0 * M_PI / 180.0) << ","
           << "\"Dec\":" << (dec_jnow * M_PI / 180.0)
           << "}";

    if (!sendCommand("SyncToRaDec", "Mount", params.str())) {
        logStream(MESSAGE_ERROR) << "setTo: SyncToRaDec failed" << sendLog;
        operationActive = false;
        return -1;
    }

    return 0;
}

int Origin::correct(double cor_ra, double cor_dec, double real_ra, double real_dec)
{
    (void)cor_ra;
    (void)cor_dec;
    return setTo(real_ra, real_dec);
}

/* -------------------------
 * Connection lifecycle
 * ------------------------- */

void Origin::updateConnectionPolicy()
{
    time_t now = time(nullptr);

    // Initialisation: MUST stay connected until siteLocationSet.
    if (initialising && !siteLocationSet) {
        if (!connected) {
            if (debugEnabled) logStream(MESSAGE_INFO) << "WS policy: initialising -> ensureConnected()" << sendLog;
            ensureConnected();
        }
        return;
    }

    // After site is known, we can disconnect if idle.
    if (!connected) {
        // reconnect if needed by activity
        if (operationActive || gotoInProgress) {
            if (debugEnabled) logStream(MESSAGE_INFO) << "WS policy: reconnect for active op" << sendLog;
            ensureConnected();
        }
        return;
    }

    // connected:
    if (operationActive || gotoInProgress) {
        return; // keep alive
    }

    // Ready/Idle: disconnect if no activity for N seconds
    const int IDLE_DISCONNECT_SEC = 12; // deliberately short for testing; adjust later
    if (now - lastActivityTime > IDLE_DISCONNECT_SEC) {
        logStream(MESSAGE_DEBUG) << "WS policy: idle timeout (" << IDLE_DISCONNECT_SEC << "s) -> disconnect" << sendLog;
        disconnectFromTelescope();
    }
}

bool Origin::ensureConnected()
{
    if (connected && webSocket && webSocket->isConnected())
        return true;
    return connectToTelescope();
}

bool Origin::connectToTelescope()
{
    if (webSocket) {
        delete webSocket;
        webSocket = nullptr;
    }

    webSocket = new OriginWebSocket();

    logStream(MESSAGE_INFO) << "Connecting to telescope at " << telescopeHost << ":" << telescopePort << sendLog;

    if (!webSocket->connect(telescopeHost, telescopePort, "/SmartScope-1.0/mountControlEndpoint")) {
        logStream(MESSAGE_ERROR) << "WebSocket connection failed" << sendLog;
        delete webSocket;
        webSocket = nullptr;
        connected = false;
        return false;
    }

    connected = true;
    lastActivityTime = time(nullptr);

    // Always ask for status immediately on connect
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
    if (!webSocket || !webSocket->isConnected())
        return;

    int msgCount = 0;
    while (webSocket->hasData() && msgCount < 200) {
        std::string msg = webSocket->receiveText();
        if (!msg.empty()) {
            lastRxTime = time(nullptr);
            processMessage(msg);
            msgCount++;
        } else {
            break;
        }
    }

    // Periodic status request:
    // - during init: frequent
    // - otherwise: modest
    time_t now = time(nullptr);
    int period = (initialising && !siteLocationSet) ? 1 : 2;
    if ((operationActive || gotoInProgress || initialising) && now - lastStatusUpdate >= period) {

    sendCommand("GetStatus", "Mount");
    lastStatusUpdate = now;
    }
}

bool Origin::sendCommand(const std::string& command,
                         const std::string& destination,
                         const std::string& params)
{
    if (!webSocket || !webSocket->isConnected()) {
        logStream(MESSAGE_ERROR) << "sendCommand: not connected (cmd=" << command << ")" << sendLog;
        return false;
    }

    std::map<std::string, std::string> cmdData;
    cmdData["Command"]     = command;
    cmdData["Destination"] = destination;
    cmdData["SequenceID"]  = std::to_string(nextSequenceId++);
    cmdData["Source"]      = "RTS2";
    cmdData["Type"]        = "Command";

    std::string message = SimpleJSON::create(cmdData);

    if (!params.empty()) {
        if (!message.empty() && message.back() == '}')
            message.pop_back();

        std::string p = params;
        if (!p.empty() && p.front() == '{')
            p.erase(p.begin());

        message += ",";
        message += p;
    }

    lastTxTime = time(nullptr);
    lastActivityTime = time(nullptr);

    if (debugEnabled) {
        logStream(MESSAGE_INFO) << "WS SEND: " << message << sendLog;
    }

    bool ok = webSocket->sendText(message);
    if (!ok) {
        logStream(MESSAGE_ERROR) << "WebSocket send FAILED (cmd=" << command << ")" << sendLog;
    }
    return ok;
}

/* -------------------------
 * Message handling & diagnostics
 * ------------------------- */

static std::vector<std::string> splitJsonObjects(const std::string& s)
{
    std::vector<std::string> out;
    int depth = 0;
    bool inStr = false;
    char q = 0;
    size_t start = std::string::npos;

    for (size_t i = 0; i < s.size(); ++i) {
        char c = s[i];

        if (inStr) {
            if (c == '\\') { // skip escaped char
                if (i + 1 < s.size()) i++;
                continue;
            }
            if (c == q) { inStr = false; q = 0; }
            continue;
        } else {
            if (c == '"' || c == '\'') { inStr = true; q = c; continue; }
        }

        if (c == '{') {
            if (depth == 0) start = i;
            depth++;
        } else if (c == '}') {
            if (depth > 0) depth--;
            if (depth == 0 && start != std::string::npos) {
                out.emplace_back(s.substr(start, i - start + 1));
                start = std::string::npos;
            }
        }
    }

    // If we failed to find balanced objects, fall back to whole string
    if (out.empty() && s.find('{') != std::string::npos)
        out.push_back(s);

    return out;
}

void Origin::processMessage(const std::string& message)
{
    // Split concatenated objects: {...}{...}{...}
    for (const auto& obj : splitJsonObjects(message)) {

        auto data = SimpleJSON::parse(obj);
        const std::string source  = (data.count("Source")  ? data["Source"]  : "");
        const std::string command = (data.count("Command") ? data["Command"] : "");
        const std::string type    = (data.count("Type")    ? data["Type"]    : "");

        if (rawFrameLogging && source == "Mount") {
            logStream(MESSAGE_INFO) << "WS RECV (Mount) raw=" << obj << sendLog;
        }

        if (type == "Error" || type == "error") {
            std::string error_msg = (data.count("Message") ? data["Message"] :
                                    (data.count("ErrorMessage") ? data["ErrorMessage"] : "Unknown error"));
            logStream(MESSAGE_ERROR)
                << "Error from " << source << " command=" << command << ": " << error_msg
                << sendLog;
            continue;
        }

        if (source == "Mount") {
            updateTelescopeStatus(obj);
        }
    }
}

void Origin::updateTelescopeStatus(const std::string& jsonData)
{
    static bool announcedSite = false;
    static bool announcedAlign = false;
    static bool announcedCoords = false;

    auto data = SimpleJSON::parse(jsonData);

    // If parser produced very few keys but raw has obvious tokens, call that out
    if (debugEnabled) {
        if (data.size() < 3 && (jsonData.find("Latitude") != std::string::npos ||
                                jsonData.find("Longitude") != std::string::npos ||
                                jsonData.find("IsAligned") != std::string::npos)) {
            logStream(MESSAGE_WARNING)
                << "Parser extracted only " << (int)data.size()
                << " keys from a Mount frame that *looks* structured; possible format mismatch"
                << sendLog;
        }
    }

    auto getD = [&](const char* k, double& out) -> bool {
        auto it = data.find(k);
        if (it == data.end())
            return false;
        if (SimpleJSON::isNullLike(it->second))
            return false;
        try {
            out = std::stod(it->second);
            return std::isfinite(out);
        } catch (...) {
            return false;
        }
    };

    auto getB = [&](const char* k, bool& out) -> bool {
        auto it = data.find(k);
        if (it == data.end()) return false;
        out = (it->second == "true" || it->second == "1");
        return true;
    };

    // Alignment
    bool alignedNow = status->isAligned;
    (void)getB("IsAligned", alignedNow);
    if (alignedNow && !status->isAligned && !announcedAlign) {
        logStream(MESSAGE_INFO) << "Mount alignment complete — sky model is now valid" << sendLog;
        // enable tracking in RTS2
        setTracking(1, false, true);
        announcedAlign = true;
    }
    status->isAligned = alignedNow;

    // Lat/Lon (key diagnostic)
    double latRad = NAN, lonRad = NAN;
    bool haveLat = getD("Latitude", latRad);
    bool haveLon = getD("Longitude", lonRad);

    if (debugEnabled) {
        // Explicitly state why we didn't accept them
        if ((data.count("Latitude") || data.count("Longitude")) && (!haveLat || !haveLon)) {
            logStream(MESSAGE_WARNING)
                << "Mount frame contained Latitude/Longitude keys but values were rejected"
                << " (Latitude='" << (data.count("Latitude") ? data["Latitude"] : "<missing>") << "'"
                << " Longitude='" << (data.count("Longitude") ? data["Longitude"] : "<missing>") << "')"
                << sendLog;
        }
        // If raw contains Latitude but parser didn't
        if ((jsonData.find("Latitude") != std::string::npos || jsonData.find("Longitude") != std::string::npos) &&
            (!data.count("Latitude") || !data.count("Longitude"))) {
            logStream(MESSAGE_WARNING)
                << "Raw frame contains 'Latitude/Longitude' text but parser did not extract both keys"
                << sendLog;
        }
    }

    if (haveLat && haveLon && !siteLocationSet) {
        double latDeg = rad2deg(latRad);
        double lonDeg = rad2deg(lonRad);

        setTelLongLat(lonDeg, latDeg);
        setTelAltitude(50.0);
	cos_lat = cos(deg2rad(latDeg));
	sin_lat = sin(deg2rad(latDeg));
	tan_lat = tan(deg2rad(latDeg));

        siteLocationSet = true;
        initialising = false;

        logStream(MESSAGE_INFO)
            << "Site location received from mount: lat=" << latDeg << " lon=" << lonDeg
            << sendLog;

        logStream(MESSAGE_INFO) << "RTS2 telescope site initialised (from mount)" << sendLog;
        announcedSite = true;
    }

    // RA/Dec: only trust once aligned (your stated requirement)
    double raRad = NAN, decRad = NAN;
    bool haveRa = getD("Ra", raRad);
    bool haveDec = getD("Dec", decRad);

    if (haveRa && haveDec && status->isAligned) {
        status->raPosition = raRad;
        status->decPosition = decRad;

        double ra_deg  = rad2deg(raRad);
        double dec_deg = rad2deg(decRad);

        setTelRaDec(ra_deg / 15.0, dec_deg);

        if (!announcedCoords && debugEnabled) {
            logStream(MESSAGE_INFO)
                << "Sky coordinates locked: RA=" << (ra_deg / 15.0) << "h DEC=" << dec_deg << "deg"
                << sendLog;
            announcedCoords = true;
        }
    }

    // Tracking/slewing
    bool trackingNow = status->isTracking;
    (void)getB("IsTracking", trackingNow);
    status->isTracking = trackingNow;

    // Origin convention you've used: IsGotoOver false => slewing true
    if (data.find("IsGotoOver") != data.end())
        status->isSlewing = (data["IsGotoOver"] == "false");

    // Battery
    if (data.find("BatteryVoltage") != data.end()) {
        try { status->batteryVoltage = std::stod(data["BatteryVoltage"]); }
        catch (...) {}
    }

    status->lastUpdate = time(nullptr);
}

/* -------------------------
 * Precession helpers
 * ------------------------- */

void Origin::j2000ToJNow(double ra_j2000, double dec_j2000, double* ra_jnow, double* dec_jnow)
{
    double jd = ln_get_julian_from_sys();

    ln_equ_posn pos_j2000, pos_jnow;
    pos_j2000.ra  = ra_j2000 * 15.0;
    pos_j2000.dec = dec_j2000;

    ln_get_equ_prec2(&pos_j2000, 2451545.0, jd, &pos_jnow);

    *ra_jnow  = pos_jnow.ra / 15.0;
    *dec_jnow = pos_jnow.dec;
}

void Origin::jnowToJ2000(double ra_jnow, double dec_jnow, double* ra_j2000, double* dec_j2000)
{
    double jd = ln_get_julian_from_sys();

    ln_equ_posn pos_jnow, pos_j2000;
    pos_jnow.ra  = ra_jnow * 15.0;
    pos_jnow.dec = dec_jnow;

    ln_get_equ_prec2(&pos_jnow, jd, 2451545.0, &pos_j2000);

    *ra_j2000  = pos_j2000.ra / 15.0;
    *dec_j2000 = pos_j2000.dec;
}

/* -------------------------
 * UDP Discovery (unchanged)
 * ------------------------- */

bool Origin::discoverTelescope()
{
    if (!startDiscovery()) return false;

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
    if (!discovering || discoverySocket < 0) return;

    char buffer[4096];
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);

    while (discoverySocket >= 0) {
        ssize_t bytesRead = recvfrom(discoverySocket, buffer, sizeof(buffer) - 1, 0,
                                     (struct sockaddr*)&sender_addr, &sender_len);
        if (bytesRead < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            logStream(MESSAGE_ERROR) << "recvfrom error: " << strerror(errno) << sendLog;
            break;
        }
        if (bytesRead == 0) break;

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

            if (extractedIP.empty()) extractedIP = sender_ip;

            logStream(MESSAGE_INFO) << "Discovered Celestron Origin at " << extractedIP << sendLog;
            telescopeHost = extractedIP;
            return;
        }
    }
}

int main(int argc, char** argv)
{
    Origin device(argc, argv);
    return device.run();
}
