# Insight SDK

Host-side SDK for Looper Robotics Insight cameras. It has one folder per
camera connection; use the one that matches how your camera is connected.

| Folder | Cameras | Connection | What it does | Guide |
|---|---|---|---|---|
| `poe/` | Insight 9P | Ethernet (PoE) | Discovery, network configuration, Fast DDS profile generation, image / depth / IMU / VIO streaming; `looper-cli` | `poe/doc/README.md`, quick start: `poe/doc/QUICKSTART.md` and the PDF |
| `uvc/` | Insight 9 | USB (UVC) | Image / IMU / VIO capture, camera parameters, calibration | `uvc/README.md` |

Each folder holds the headers (`include/`), the libraries (`lib/`, and `bin/`
for programs and Windows DLLs), example sources (`examples/`) and its guide.
The two are independent: neither needs the other.

## PoE

```sh
poe/bin/looper-cli discover                                 # cameras on the network
cmake -S poe/examples -B build-poe -DCMAKE_PREFIX_PATH=$PWD/poe
cmake --build build-poe -j
```

In your own CMake project:

```cmake
find_package(LooperSDK 1.0 REQUIRED)    # -DCMAKE_PREFIX_PATH=<this folder>/poe
target_link_libraries(app PRIVATE LooperSDK::looper)
```

On Windows, copy `poe/bin/looper.dll` next to your executable; MSVC links
through `poe/lib/looper.lib`, MinGW through `poe/lib/looper.dll.a`.

## UVC

```sh
cmake -S uvc/examples -B build-uvc -DCMAKE_BUILD_TYPE=Release   # needs OpenCV
cmake --build build-uvc -j
sudo ./build-uvc/example
```

## Debian package

The deb installs the same under `/usr`: headers in `/usr/include`, libraries
in `/usr/lib`, `looper-cli` in `/usr/bin`, examples in
`/usr/share/insightsdk/poe/examples` and `/usr/share/insightsdk/uvc/examples`,
guides in `/usr/share/doc/insightsdk`.

## License

Headers, examples and documentation are under the MIT license; the libraries
and programs are provided in binary form under the Looper Robotics binary
license. See `LICENSE.txt`, and `licenses/` for third party software.
