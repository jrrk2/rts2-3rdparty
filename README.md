# RTS2 Drivers for Celestron Origin

RTS2 is an observatory automation package in C++. This repo supplements that with specific
drivers for Celestron Origin (alpha status)

1. **rts2-teld-origin** - Telescope/mount driver
2. **rts2-camd-origin** - Camera driver

Both drivers communicate directly with the Celestron Origin via Ethernet/WebSocket, eliminating the need for INDI.

## Features

### Telescope Driver (rts2-teld-origin)
- GOTO and SYNC commands
- Tracking control
- Park/Unpark functionality
- Coordinate precession (J2000 ↔ JNow)
- Real-time status monitoring
- Battery voltage monitoring

### Camera Driver (rts2-camd-origin)
- Full frame and preview mode imaging
- ISO/Gain control
- Binning support
- Automatic image download
- FITS image handling

## Architecture

```
┌─────────────┐                    ┌─────────────────────┐
│    RTS2     │                    │  Celestron Origin   │
│  Scheduler  │                    │    (WebSocket)      │
└──────┬──────┘                    └──────────┬──────────┘
       │                                      │
       ├──────────────────────────────────────┤
       │                                      │
┌──────▼──────────┐                 ┌─────────▼──────────┐
│  rts2-teld-     │◄───WebSocket───►│   Mount Control    │
│    origin       │                 │                    │
└─────────────────┘                 └────────────────────┘
       │                                      │
┌──────▼──────────┐                  ┌────────▼───────────┐
│  rts2-camd-     │◄───WebSocket────►│  Camera Control    │
│    origin       │    + HTTP        │                    │
└─────────────────┘                  └────────────────────┘
```

## Prerequisites

### Required Libraries
- RTS2 development files
- OpenSSL (for WebSocket)
- libcurl (for image download)
- libnova (for coordinate transformations)
- libcfitsio (for FITS handling)

### Installation
```bash
# Ubuntu/Debian
sudo apt-get install libssl-dev libcurl4-openssl-dev libnova-dev libcfitsio-dev

# The system should already have RTS2 installed
```

## Building

### Using Make

```bash
# Set environment
export CPATH=/home/jonathan/src/rts2/include

# Build
make -f Makefile-origin

# Install
sudo make -f Makefile-origin install
```

### Using CMake

```bash
# Set environment
export CPATH=/home/jonathan/src/rts2/include

# Build
mkdir build
cd build
cmake -f ../CMakeLists-origin.txt ..
make

# Install
sudo make install
```

## Configuration

### RTS2 Device Configuration

```ini
[general]
logdir = /home/jonathan/rts2/log
rundir = /home/jonathan/rts2/run

[xmlrpc]
port = 8889
; Optional: add authentication if desired
; http_username = rts2
; http_password = password

[observatory]
name = testobs
latitude = 52.26
longitude = 0.07
altitude = 50
day_horizon = 0
night_horizon = -6
evening_time = 0
morning_time = 0
# ✅ Add flat time settings for selector
flat_sun_min = -2
flat_sun_max = -2

[database]
name = rts2

[devices]
C0 = camd         # ✅ Add C0
T0 = teld         # ✅ Add T0
W0 = sensor
IMGP = imgproc    # ✅ Fix typo
selector = selector    # ✅ Add selector
E0 = executor   # ✅ Add executor

[C0]
type = camd
name = C0
exec = /usr/local/bin/rts2-camd-dummy
args = --debug --gentype 6 --width 1024 --height 1024
instrume = DUMMY-C0
telescop = DUMMY-T0
origin = RTS2-SIMULATION
template = /etc/rts2/template.fits
script = default

[T0]
type = teld
name = T0
exec = /usr/local/bin/rts2-teld-dummy
args = --debug

[IMGP]
type = imgproc
name = IMGP
exec = /usr/local/bin/rts2-imgproc

# ✅ Add selector config
[selector]
type = selector
name = SEL
exec = /usr/local/bin/rts2-selector
select_period = 60

# ✅ Add executor config
[E0]
type = executor
name = EXEC
exec = /usr/local/bin/rts2-executor
default_script = "E 1"

[centrald]
name = demo
ignore_weather = true
auto_on = true

[executor]
# ✅ Tell executor to process images
astrometry = true
astrometry_timeout = 120

[imgproc]
astrometry = /usr/local/bin/rts2-astrometry-wrapper
obsprocess = 
imageglob = /tmp/*.fits                    # ✅ Use wildcard, not specific file
camera = C0
process_images = true
queue_only = false                          # ✅ CRITICAL FIX: was true, should be false
check_interval = 5
```

### Command Line Usage

**Telescope:**
```bash
rts2-teld-origin -a 192.168.1.100 -d T0
```

**Camera:**
```bash
rts2-camd-origin -a 192.168.1.100 -d C0
```

Options:
- `-a <address>` - IP address of Celestron Origin (required)
- `-p <port>` - Port number (default: 80)
- `-d <device>` - RTS2 device name

## Network Discovery

The Celestron Origin broadcasts UDP discovery packets on port 55555. The drivers can automatically discover the telescope:

### Using Discovery

```bash
# Just add the -D flag - no IP address needed!
rts2-teld-origin -D -d T0
```

The driver will:
1. Listen on UDP port 55555 for telescope broadcasts
2. Wait up to 30 seconds for discovery
3. Automatically connect to the first Origin found
4. Display the discovered IP address

### Manual Discovery

You can also manually find your telescope:

```bash
# Listen for discovery packets
sudo tcpdump -i any -A udp port 55555

# Or use netcat
nc -u -l 55555
```

The Origin broadcasts packets containing:
- Device model: "Celestron Origin"  
- IP Address
- Other device information

### Troubleshooting Discovery

If discovery fails:

1. **Check network connection**: Ensure your computer is on the same network as the Origin
2. **Check firewall**: UDP port 55555 must be open for incoming broadcasts
3. **Use manual IP**: Specify the IP directly with `-a <ip>` if discovery doesn't work
4. **Check Origin is powered**: The telescope must be on and connected to the network

## Usage Examples

### With Auto-Discovery (Recommended)

```bash
# Start both drivers with auto-discovery
rts2-teld-origin -D -d T0 &
rts2-camd-origin -D -d C0 &
```

The drivers will automatically discover your Celestron Origin on the network via UDP broadcasts (port 55555).

### With Manual IP Address

```bash
# Start telescope daemon with specific IP
rts2-teld-origin -a 192.168.1.100 -d T0 &

# Start camera daemon with specific IP
rts2-camd-origin -a 192.168.1.100 -d C0 &
```

### Command Line Options

**Telescope:**
```bash
rts2-teld-origin [OPTIONS]
  -a <address>  Telescope IP address (optional if using -D)
  -p <port>     Telescope port (default: 80)
  -D            Auto-discover telescope on network
  -d <device>   RTS2 device name (e.g., T0)
```

**Camera:**
```bash
rts2-camd-origin [OPTIONS]
  -a <address>  Telescope IP address (optional if using -D)
  -p <port>     Telescope port (default: 80)
  -D            Auto-discover telescope on network
  -d <device>   RTS2 device name (e.g., C0)
```

# Use RTS2 commands
rts2-mon

# In RTS2 monitor, execute:
# C0 exposure 10   # 10 second exposure
# T0 move 12:30:00 +45:00:00  # GOTO coordinates
```

### Integration with RTS2 Scheduler

The drivers integrate seamlessly with RTS2's scheduler for automated observations:

```
# Example target list
1 NGC7000 "20:59:17.14" "+44:31:43.6" 
2 M31 "00:42:44.3" "+41:16:9"
```

## Protocol Details

### WebSocket Communication

Both drivers communicate with the Origin via WebSocket at:
- **URL:** `ws://<telescope-ip>/SmartScope-1.0/mountControlEndpoint`
- **Protocol:** JSON-based command/response

### Message Format

```json
{
  "Sequence": 1,
  "Source": "RTS2",
  "Destination": "Mount",
  "Command": "GotoRaDec",
  "Type": "Command",
  "Ra": 3.14159,
  "Dec": 0.7854
}
```

### Image Download

Images are downloaded via HTTP:
- **URL:** `http://<telescope-ip>/<FileLocation>`
- **Format:** FITS or TIFF

## Coordinate Systems

- **RTS2 uses J2000.0** coordinates internally
- **Origin mount uses J2000** coordinates
- Drivers automatically handle precession using libnova

## Troubleshooting

This driver does not handle initialisation, this should be done
by the OriginApp or using a utility such as OriginMonitor (available in source form)

### Connection Issues

2. **Verify WebSocket connection:**
   ```bash
   # Check if port 80 is accessible
   telnet 192.168.1.100 80
   ```

3. **Enable debug logging:**
   Add `-v` or `--debug` flags to driver commands

### Image Download Failures

- Ensure sufficient disk space
- Check network bandwidth
- Try preview mode for faster downloads (smaller files)

### Coordinate Mismatches

- Verify telescope alignment
- Check that mount reports `IsAligned: true`
- Sync to a known star before observations

## Implementation Notes

### Threading Model
- Both drivers are single-threaded
- WebSocket polling occurs during RTS2's `info()` calls
- Non-blocking I/O throughout

### Memory Management
- Image buffers dynamically allocated
- Automatic cleanup on driver shutdown
- No Qt dependencies (pure C++ with standard library)

### Error Handling
- Graceful WebSocket reconnection
- Timeout protection on all network operations
- Comprehensive RTS2 logging

## Development

### Adding New Commands

1. Add command to `sendCommand()` method
2. Handle response in `processMessage()`
3. Update status structures as needed

### Extending Functionality

The modular design allows easy extension:
- `origin-websocket.cpp` - WebSocket protocol
- `origin-teld.cpp` - Telescope control
- `origin-camd.cpp` - Camera control
- `origin-data.h` - Data structures

## License

GPL v2+ (compatible with RTS2)

## Credits

Refactored from INDI driver to native RTS2 implementation.
Original INDI driver structure preserved where applicable.

## Support

For issues specific to:
- **RTS2 integration:** RTS2 mailing lists
- **Celestron Origin:** Celestron support
- **Driver bugs:** Report to your repository issues

## See Also

- RTS2 Documentation: http://www.rts2.org/
- Celestron Origin: https://www.celestron.com/
- WebSocket RFC: https://tools.ietf.org/html/rfc6455
