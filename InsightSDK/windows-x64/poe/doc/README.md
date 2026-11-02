# Insight SDK - PoE cameras

Discovery, network configuration and streaming for Insight PoE network
cameras (Insight 9P). This is the `poe/` folder of the Insight SDK.

This module provides a C API, which is the stable ABI, and a header-only C++
wrapper on top of it.

Setting up a camera for the first time: **[QUICKSTART.md](QUICKSTART.md)**
(Chinese, next to this guide, also as a PDF) takes a field engineer from power-on through addressing, time
synchronisation and subscribing to topics.

## Contents

`poe/` in the SDK package (`InsightSDK-<ver>-<platform>.tar.gz` / `.zip`):

```
include/looper/looper.h          C API: discovery, network configuration, DDS profile
include/looper/looper.hpp        C++ wrapper (RAII, exceptions)
include/looper/stream.h          C API: streaming (images, depth, IMU, VIO pose)
include/looper/stream.hpp        C++ wrapper: Device, Frame, Pipeline, FrameSet
lib/                             liblooper.so.1 (Linux);
                                 looper.lib (MSVC), looper.dll.a (MinGW) (Windows)
lib/cmake/LooperSDK/             CMake package (find_package)
bin/looper-cli                   command line tool; looper.dll on Windows
examples/                        example sources, see below
doc/                             this guide and the quick start guide
```

The deb (`InsightSDK-<ver>-linux-<arch>.deb`, package `insightsdk`) installs
these under `/usr`: `/usr/include/looper`, `/usr/lib`, `/usr/bin/looper-cli`,
the examples in `/usr/share/insightsdk/poe/examples` and the guides in
`/usr/share/doc/insightsdk/poe`.

Examples:

```
0.basic.enumerate   list the cameras on the network
1.device.forceip    assign an address to a camera
2.dds.profile       restrict Fast DDS to the right interface
3.stream.imu_pose   IMU and VIO pose through callbacks
4.stream.frameset   time-matched IR / depth / color sets, saved to disk
5.stream.viewer     live view (built only when OpenCV is found)
```

## Requirements

- Linux (x86_64, aarch64), or Windows 7 or later (MSVC or MinGW-w64)
- To build the examples or your application: CMake 3.10 or later, a C99 and
  C++11 compiler

No third-party dependencies. The optional viewer example needs OpenCV.

### Use in your project

The SDK provides a CMake package with the target `LooperSDK::looper`, the
shared library (`liblooper.so.1`, `looper.dll`).

```cmake
find_package(LooperSDK 1.0 REQUIRED)       # -DCMAKE_PREFIX_PATH=<SDK>/poe if not in /usr
target_link_libraries(app PRIVATE LooperSDK::looper)
```

- On Linux, `pkg-config --cflags --libs looper` works as well.
- On Windows, copy `bin/looper.dll` next to your executable.
  - MSVC links through `lib/looper.lib`, MinGW through `lib/looper.dll.a`;
    the CMake package selects the right one.
- The examples build against the SDK as they are, from the `poe/` folder:
  `cmake -S examples -B build -DCMAKE_PREFIX_PATH=$PWD`.
- `lp_version()` returns the library's version, and `LP_VERSION_STRING` the
  header's version.

## Quick start

### Command line

From the `poe/` folder; with the deb, `looper-cli` is on the PATH.

```sh
# List cameras on the network
bin/looper-cli discover

# On a host with several network interfaces, choose the one to use
bin/looper-cli discover --bind 192.168.1.1

# Persistent static address, kept across reboots
bin/looper-cli set-static 00:11:22:33:44:55 192.168.1.50 255.255.255.0 192.168.1.1

# Temporary address, reverts on the next reboot
bin/looper-cli set-temp 00:11:22:33:44:55 192.168.1.99

# Return the camera to DHCP
bin/looper-cli set-dhcp 00:11:22:33:44:55

# Generate a Fast DDS profile for the interface that reaches the cameras
bin/looper-cli dds-profile --out looper_fastdds_profile.xml

# Camera description and calibration (streaming mode)
bin/looper-cli info 192.168.1.10

# Receive streams and print rates once a second
bin/looper-cli stream 192.168.1.10 --streams ir_left,depth,imu,pose

# Optional, once per host: raise net.core.rmem_max for good
sudo bin/looper-cli setup-host
```

### C++

```cpp
#include <looper/looper.hpp>

lp::Context ctx;                                  // or ctx("192.168.1.1")
for (const auto &d : ctx.discover()) {
    std::cout << d.mac << " " << d.ip << " " << d.ipModeString() << "\n";
}
ctx.forceIp("00:11:22:33:44:55", lp::IpConfig::Static("192.168.1.50"));
```

### C

```c
#include <looper/looper.h>

lp_context *ctx = lp_context_create(NULL);
lp_device_info devices[16];
size_t found = 0;
if (lp_context_discover(ctx, devices, 16, &found, 1000) == LP_OK) {
    for (size_t i = 0; i < found; i++) {
        printf("%s %s\n", devices[i].mac, devices[i].ip);
    }
}
lp_context_destroy(ctx);
```

See `include/looper/looper.h` for the full API reference.

## How it works

Discovery and address configuration use UDP broadcast on port 19800, and a
camera is addressed by its **MAC address**. This means a camera can be found
and reconfigured even when its current IP address is not reachable from the
host, for example when it is on another subnet.

After changing a camera's address, run discovery again to confirm the change.

### Address modes

The mode reported by `discover`:

| mode | meaning |
|------|---------|
| `dhcp` | address obtained via DHCP |
| `static` | static address configured |
| `link-local` | DHCP failed and the configured static address was already in use, so the camera fell back to a 169.254.x.x address |

A camera in `link-local` mode usually means two cameras share the same static
address. Assign each camera a unique address.

## Fast DDS configuration

The cameras publish data over ROS 2. By default Fast DDS announces one locator
per local network interface, and each camera sends a copy of every sample to
each announced locator. On a host with several interfaces (for example
Ethernet, Wi-Fi and a VPN), this multiplies the bandwidth each camera has to
send.

The SDK can generate a Fast DDS profile that restricts DDS to the interface(s)
that actually reach the cameras:

```sh
bin/looper-cli dds-profile --bind 192.168.1.1 --out looper_fastdds_profile.xml
export FASTRTPS_DEFAULT_PROFILES_FILE=/abs/path/looper_fastdds_profile.xml
```

The command discovers the cameras, determines which local address routes to
each of them, and writes those addresses to the profile. It prints the
absolute path to export.

Or from C++:

```cpp
lp::Context ctx;
const auto devices = ctx.discover();
lp::writeDdsProfile("looper_fastdds_profile.xml", lp::localAddressesFor(devices));
```

Notes:

- **Use `--bind` when the cameras are not on the default route.** Discovery
  broadcasts leave through the default route unless an interface is chosen,
  so on a laptop whose default route is Wi-Fi, cameras connected via Ethernet
  will not be found.
- **Export the variable before starting the subscriber.** Fast DDS reads the
  profile when a participant is created; running processes, including the
  ROS 2 daemon, will not pick it up (run `ros2 daemon stop` first).
- **Raise the socket buffer limit.** The profile requests an 8 MB socket
  buffer, which the kernel caps at `net.core.rmem_max`. A small limit can
  cause dropped packets. `looper-cli dds-profile` warns when the limit is too
  low:

  ```sh
  sudo sysctl -w net.core.rmem_max=8388608
  ```

No change on the camera side is needed.

## Streaming

Images, depth, IMU and VIO pose can be received straight from the camera,
without ROS. The transport has three planes:

| Plane | Transport | Notes |
|---|---|---|
| Discovery | UDP 19800 broadcast | the `discover` above; gives the IP to open |
| Control | TCP 8090 | one host session per camera, kept alive by a heartbeat every second; the camera stops streaming after 3 s of silence |
| Data | one RTP/UDP flow per stream | camera to a host port chosen per stream; a frame with a lost packet is dropped, nothing is retransmitted |

| Stream | Format | Rate (insight9P) |
|---|---|---|
| `ir_left`, `ir_right` | 8-bit mono, rectified (raw in calibration mode) | stereo frame rate |
| `depth` | uint16 millimetres, 0 = no depth, aligned to rectified IR left | ~10 Hz |
| `color` | MJPEG | 30 Hz |
| `imu` | `lp_imu_data`: acc m/s², gyr rad/s | ~400 Hz |
| `pose` | `lp_pose_data`: position, quaternion xyzw, VIO status | ~100 Hz |

Streaming needs the camera to run in Ethernet streaming mode;
`lp_device_open()` reports "not in streaming mode" when
it is not.

### C++

Per-stream callbacks, which suit IMU and pose:

```cpp
#include <looper/stream.hpp>

lp::Device dev("192.168.1.10");
dev.start(lp::Stream::Imu, [](const lp::Frame &f) {
    lp::ImuData d = f.imu();      // runs on the stream's receive thread
});
```

A `Pipeline` groups the image streams into time-matched `FrameSet`s. IR left, IR right and depth of one stereo capture
share a timestamp and are matched exactly; the colour frame nearest that
capture is attached.

```cpp
lp::Pipeline pipe(dev);
lp::Config   cfg;
cfg.enableStream(lp::Stream::Depth);
cfg.enableStream(lp::Stream::Color);
pipe.start(cfg);

lp::FrameSet fs = pipe.waitForFrames(1000);
if(fs) {
    uint16_t mm = fs.depth().depthAt(320, 240);
}

lp::Intrinsics in = dev.intrinsics(lp::Stream::Depth);  // in.p: rectified projection
```

### C

```c
#include <looper/stream.h>

static void on_frame(const lp_frame *f, void *user) {
    /* f and f->data are valid only during this call; lp_frame_clone() to keep */
}

lp_device *dev = NULL;
if(lp_device_open("192.168.1.10", 2000, &dev) != LP_OK) {
    fprintf(stderr, "%s\n", lp_open_error());
}
lp_device_start_stream(dev, LP_STREAM_DEPTH, 0, on_frame, NULL);
...
lp_device_close(dev);
```

See `include/looper/stream.h` for the full reference. For a ready-made
graphical client, see InsightViewer.

### Notes

- **Socket buffer.** The camera paces its output, so the default receive
  buffer (about 400 KB) is enough at typical rates (measured: no drops at
  ~190 Mbps). For headroom on busy hosts or near-gigabit rates, the SDK asks
  for a large buffer with `SO_RCVBUFFORCE`, which works when the process has
  `CAP_NET_ADMIN`, and otherwise falls back to the `net.core.rmem_max` limit.
  `lp_stream_stats::socket_buffer` reports what was granted, and
  `looper-cli stream` prints a note if frames drop with a small buffer. Either
  of these, once, gives the headroom:

  ```sh
  sudo looper-cli setup-host                        # raise rmem_max, persisted in /etc/sysctl.d
  sudo setcap cap_net_admin+ep ./your_application   # or: let one binary bypass the limit
  ```

- **Host firewalls.** The streams are UDP sent by the camera, which a
  firewall on the host (Windows Defender, ufw) would drop as unsolicited.
  The camera reports the port each stream leaves from, and the SDK sends it a
  small datagram from the receive port every second; the firewall then takes
  the stream for the answer to it, so no rule is needed.
  `lp_stream_stats::source_port` is that port. It is 0 with camera firmware
  from before this, and then the application needs an inbound rule, e.g. on
  Windows (administrator PowerShell):

  ```powershell
  New-NetFirewallRule -DisplayName looper -Direction Inbound -Action Allow -Profile Any -Program "C:\path\to\app.exe"
  ```

- **Callbacks run on receive threads**, one per stream. Return quickly, and do
  not stop streams or close the device from inside one.
- **Several cameras on one host** work: each stream takes a free UDP port
  unless one is given. Each camera accepts one host at a time; a second one
  gets `LP_ERR_BUSY`.
- **Timestamps.** `timestamp_ns` (C) / `timestamp()` (C++) is the capture time
  on the camera's monotonic clock, shared by all its streams. The system
  timestamp is the same instant on the camera's wall clock, comparable across
  cameras only when their clocks are synchronised (`CLOCK` in `discover`).
- **Bandwidth.** Two 544×640 IR streams at 20 fps take about 112 Mbps, depth
  about 60 Mbps, color about 15 Mbps; IMU and pose are negligible.
- **A lost session is not resumed.** After a cable pull or camera restart,
  `lp_device_is_connected()` turns 0; close the device and open it again.

## Scope

This release covers camera discovery, network configuration, the Fast DDS
profile helper, and streaming in Ethernet streaming mode. Camera property
control (exposure, resolution and so on) is not part of the SDK yet.
