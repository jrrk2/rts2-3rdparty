/*
 * Celestron Origin Camera Driver for RTS2
 *
 * Header updated to match the new snapshot-latch based camera driver:
 *  - No imageReady flag
 *  - Explicit snapshot latch state
 *  - Thread-safe separation between WebSocket RX and RTS2 readout
 */

#ifndef __RTS2_ORIGIN_CAMD_H__
#define __RTS2_ORIGIN_CAMD_H__

#include "camd.h"

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <tiffio.h>

// Forward declarations
class OriginWebSocket;

namespace rts2camd
{

struct CameraStatus
{
    int iso;
    double exposure;
    int binning;
    double temperature;
    time_t lastUpdate;
};

class OriginCam : public Camera
{
public:
    OriginCam(int argc, char **argv);
    virtual ~OriginCam();

protected:
    // RTS2 hooks
    int processOption(int opt) override;
    int initHardware() override;
    int initChips() override;
    int info() override;

    int startExposure() override;
    int stopExposure() override;
    long isExposing() override;
    int doReadout() override;

    int setCoolTemp(float new_temp) override;
    int switchCooling(bool cooling) override;

    int commandAuthorized(rts2core::Connection *conn) override;
    virtual int initValues() override;

private:
    std::vector<uint16_t> monoFrame;   // decoded RGGB image
    // ---------------- Threading ----------------
    std::thread pollThread;
    std::atomic<bool> pollRunning{false};
    void pollLoop();

    // ---------------- Connection ----------------
    std::string telescopeHost;
    int telescopePort{80};
    bool useDiscovery{false};

    OriginWebSocket *webSocket{nullptr};
    bool connected{false};

    // ---------------- Camera state ----------------
    CameraStatus *cameraStatus{nullptr};
    int nextSequenceId{1};

    // ---------------- Exposure / Snapshot latch ----------------
    double exposureStart{0.0};
    double exposureDuration{0.0};
    bool exposureInProgress{false};

    // Snapshot latch (TIFF only)
    std::mutex snapshotMutex;
    bool snapshotArmed{false};
    bool snapshotReady{false};
    double snapshotArmTime{0.0};

    std::string pendingImagePath;
    double pendingImageRA{0.0};
    double pendingImageDec{0.0};

    // ---------------- Image handling ----------------
    unsigned char *imageBuffer{nullptr};
    size_t imageBufferSize{0};

    bool decodeTIFF(const std::vector<uint8_t> &imageData);
    std::vector<uint8_t> downloadImageSync(const std::string &url);

    // ---------------- WebSocket protocol ----------------
    bool connectToTelescope();
    void disconnectFromTelescope();

    bool sendCommand(const std::string &command,
                     const std::string &destination,
                     const std::string &params = "");

    void processMessage(const std::string &message);
    void updateCameraStatus(const std::string &jsonData);
    void handleImageReady(const std::string &filePath,
                          double ra,
                          double dec);

    // ---------------- Discovery ----------------
    bool discoverTelescope();
    bool startDiscovery();
    void stopDiscovery();
    void pollDiscovery();

    int discoverySocket{-1};
    bool discovering{false};
    time_t discoveryStartTime{0};

    // ---------------- RTS2 values ----------------
    rts2core::ValueString  *telescopeAddress{nullptr};
    rts2core::ValueInteger *gain{nullptr};
    rts2core::ValueBool    *previewMode{nullptr};
    rts2core::ValueDouble  *ccdTemp{nullptr};
};

} // namespace rts2camd

#endif // __RTS2_ORIGIN_CAMD_H__

