# Changelog

## 1.0.0 (2026-10-30)

First release of the Insight SDK and InsightViewer.

Packages: `InsightSDK-<ver>-linux-<arch>.tar.gz`, `InsightSDK-<ver>-linux-<arch>.deb`
(Debian package `insightsdk`), `InsightSDK-<ver>-windows-x64.zip`,
`InsightViewer-<ver>-linux-<arch>.deb` (Debian package `insightviewer`),
`InsightViewer-<ver>-windows-x64.zip`. The SDK has one folder per camera
connection, `poe/` and `uvc/`. License terms in `LICENSE.txt`, third party
licenses in `licenses/`.

### PoE SDK (`poe/`)

- Discovery and network configuration over UDP broadcast, addressed by MAC:
  static, temporary and DHCP addresses, also for a camera on another subnet.
- Fast DDS profile generation that pins DDS to the interfaces reaching the cameras.
- Streaming without ROS (camera in Ethernet streaming mode):
  IR left / right, depth, color (MJPEG), IMU and VIO pose; per-stream callbacks
  and time-matched `FrameSet`s.
- Host firewalls: the camera reports each stream's source port and the SDK
  keeps that path open, so no inbound rule is needed (Windows Defender, ufw).
- `looper-cli`: `discover`, `set-static` / `set-temp` / `set-dhcp`,
  `dds-profile`, `info`, `stream`, `setup-host`, `--version`.
- Shared library (`liblooper.so.1`, `looper.dll`); CMake package
  `find_package(LooperSDK)` with `LooperSDK::looper`; pkg-config `looper` on Linux.
- Linux x86_64 / aarch64 and Windows x64.

### UVC SDK (`uvc/`)

- Insight 9 over USB: image, IMU and VIO capture, camera parameters,
  calibration, depth-to-RGB alignment (`libinsight9`, see `uvc/README.md`).
- Linux; Windows to follow.

### InsightViewer

- PoE (up to 4 cameras), GMSL on the S100, `.insrec` recordings and `.gmslrec` playback.
- Automatic source selection; address clash detection and address change from the Control panel.
- Recording, playback, snapshots; Linux deb and Windows zip.
