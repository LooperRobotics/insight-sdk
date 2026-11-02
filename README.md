# Looper Robotics Insight SDK

Host-side SDK and viewer for Looper Robotics Insight cameras, as headers,
prebuilt libraries and tools, example sources and documentation.

| Folder | What it is |
|---|---|
| `InsightSDK/<platform>/` | The SDK, one folder per camera connection: `poe/` (Insight 9P, Ethernet) and `uvc/` (Insight 9, USB). Start with its `README.md`. |
| `InsightViewer/` | InsightViewer, a desktop viewer: images, depth, point cloud, IMU, VIO, calibration; recording, playback, snapshots. Packages for Linux (.deb) and Windows (.zip), and its [README](InsightViewer/README.md). |

`<platform>` is `linux-x86_64`, `linux-aarch64` or `windows-x64`, as
available. On Debian / Ubuntu, `InsightSDK/InsightSDK-<version>-linux-<arch>.deb`
installs the same SDK under `/usr`.

## Quick start

PoE camera, from power-on: `InsightSDK/<platform>/poe/doc/QUICKSTART.md`, or
the PDF quick start guide next to it.

```sh
InsightSDK/linux-x86_64/poe/bin/looper-cli discover          # PoE cameras on the network

sudo apt install ./InsightViewer/InsightViewer-<version>-linux-x86_64.deb
InsightViewer                                                # finds a camera and opens it
```

On Windows, use `InsightSDK\windows-x64\poe\bin\looper-cli.exe`, and unzip
`InsightViewer-<version>-windows-x64.zip` to run `InsightViewer.exe`.

## License

Headers, examples and documentation are under the MIT license; the libraries
and programs are provided in binary form under the Looper Robotics binary
license. See [LICENSE.txt](LICENSE.txt) and [licenses/](licenses/) for third
party software.
