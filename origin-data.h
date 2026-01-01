/*
 * Data structures for Celestron Origin
 * Qt-free implementation using standard C++
 */

#ifndef __RTS2_ORIGIN_DATA_H__
#define __RTS2_ORIGIN_DATA_H__

#include <string>
#include <ctime>

struct TelescopeStatus
{
    // Mount status
    double altitude;
    double azimuth;
    double raPosition;
    double decPosition;
    bool isAligned;
    bool isTracking;
    bool isSlewing;
    bool isParked;
    
    // Battery
    double batteryVoltage;
    double batteryCurrent;
    std::string batteryLevel;
    
    // Location/Time
    double latitude;
    double longitude;
    std::string dateTime;
    std::string timeZone;
    
    // Errors
    double altitudeError;
    double azimuthError;
    
    time_t lastUpdate;
    
    TelescopeStatus() :
        altitude(0), azimuth(0), raPosition(0), decPosition(0),
        isAligned(false), isTracking(false), isSlewing(false), isParked(false),
        batteryVoltage(0), batteryCurrent(0),
        latitude(0), longitude(0),
        altitudeError(0), azimuthError(0),
        lastUpdate(0)
    {}
};

struct CameraStatus
{
    int binning;
    int bitDepth;
    double exposure;
    int iso;
    int offset;
    double colorRBalance;
    double colorGBalance;
    double colorBBalance;
    double temperature;
    
    time_t lastUpdate;
    
    CameraStatus() :
        binning(1), bitDepth(16), exposure(0), iso(0), offset(0),
        colorRBalance(1.0), colorGBalance(1.0), colorBBalance(1.0),
        temperature(20.0),
        lastUpdate(0)
    {}
};

struct ImageInfo
{
    std::string fileLocation;
    std::string imageType;
    double ra;
    double dec;
    double orientation;
    double fovX;
    double fovY;
    
    ImageInfo() :
        ra(0), dec(0), orientation(0), fovX(0), fovY(0)
    {}
};

#endif // __RTS2_ORIGIN_DATA_H__
