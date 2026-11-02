# InsightViewer

A desktop viewer for Looper Insight cameras. It shows everything a camera sends: both IR eyes, depth, colour, the point cloud, IMU plots, the VIO trajectory, calibration and link health. It also records, plays back and takes snapshots. No ROS.

It works the same whichever way the camera is connected:

| Source | Platforms | Notes |
|---|---|---|
| PoE (Ethernet) | Linux, Windows | Through LooperSDK's streaming API. Up to 4 cameras at once. The camera must run in Ethernet streaming mode. |
| GMSL | S100 | X5s on the MAX96712. Up to 4 cameras, kept in step by the host trigger. Needs root (the launcher asks through pkexec). |
| `.insrec` recording | Linux, Windows | The viewer's own format. Any source can record to it. |
| `.gmslrec` recording | Linux, Windows | GMSL's raw containers. Played back by packages built with GMSL support. |

The layout, theme and panels are those of gmsl-viewer. A layout saved by gmsl-viewer (`~/.gmsl_viewer.ini`) is picked up on the first start.

## Which source opens

With no options, the viewer looks for a source and opens one. It runs these checks in parallel and needs about a second:

- **GMSL**: whether this host is an S100 with the deserializer. The test is the same one `gmsl-poc` uses: the device tree declares a power-over-coax pin set. The X5 itself is not probed, because it streams only about a minute after power-on.
- **PoE**: discovery broadcasts (UDP 19800) on every network interface. Each camera that answers is then asked for a stream session on TCP 8090. The answer sorts it into one of these states:
  - `ready`: in Ethernet streaming mode.
  - `ROS mode`: the connection is refused.
  - `in use`: another host holds the session.
  - `address clash`: another camera answers on the same address, so neither can be used. This is not probed.

Then:

1. If the source used last time is there again, it opens. For PoE, a `ready` camera that was not on the network last time joins the cameras used last time. A camera you left unticked stays off.
2. Otherwise, GMSL hardware with no PoE camera opens GMSL.
3. Otherwise, with no GMSL, every `ready` PoE camera opens, up to four.
4. Otherwise (GMSL and PoE both, or nothing), the Control panel lists what was found to pick from. It searches again every 3 s. You can also type a camera's address there, for a camera that discovery cannot reach.

### Several PoE cameras

- Each camera needs an address of its own. Cameras leave the factory with the same one, so the first time several share a network, the Control panel marks them `address clash`.
- To fix a clash, press **address** on a camera's row, then **Apply**.
  - The viewer proposes the next free address in the camera's subnet.
  - The change reaches the camera by its MAC, so it works while the clash lasts.
  - It is kept after the camera restarts unless you untick that.
  - `looper-cli set-static <mac> <ip>` does the same from a shell.
- Bandwidth: one camera sends about 130 Mbit/s with every stream on, so four fit on one gigabit port.

Options choose the source directly:

```
InsightViewer [source] [--scale X] [--fps N]
  --net IP[,IP...]                  PoE cameras at these addresses
  --gmsl [--link N[,N...]] [--trigger HZ] [-- <client options>]
  --play FILE [--loop] [--speed X]  .insrec or .gmslrec
```

The deb's launcher (`InsightViewer`) reads `/etc/default/InsightViewer` when given no options:

| Setting | Meaning |
|---|---|
| `INSIGHT_SOURCE=auto\|poe\|gmsl` | Which source to open. |
| `INSIGHT_POE_IPS` | The PoE addresses to open with `poe`. |
| `GMSL_LINK`, `GMSL_TRIGGER` | The GMSL links and trigger rate. |
| `INSIGHT_VIEWER_ARGS` | Further options for the viewer. |
| `INSIGHT_VIEWER_GL` | On the S100, how the viewer draws: `auto`, `gpu` or `soft`. |

## Install

```sh
sudo apt install ./InsightViewer-<ver>-linux-<arch>.deb   # Linux (package insightviewer), then run InsightViewer
```

- **amd64 deb**: runs as the user and needs no configuration.
- **arm64 deb**: built on an S100. It replaces `gmsl-viewer`:
  - its settings carry over;
  - it keeps `gmsl-poc.service`, which powers the X5 at boot;
  - it asks for root only when GMSL may be opened.
- **Windows zip**: `InsightViewer.exe` and `SDL2.dll`. Unzip and run.

## Use

- **Keys**:

  | Key | Action |
  |---|---|
  | Space | Pause or resume playback |
  | Right | Step one frame |
  | R | Record |
  | S | Snapshot |
  | 1-4 | Show or hide camera A-D |
  | 0 | Show all cameras |

  You can also drop a recording on the window to play it.
- **Record** writes `.insrec` by default. For a GMSL source you can choose `.gmslrec` instead, which keeps the raw containers.
- **Snapshot** writes the following to `~/insight_snapshot_<time>/`, one directory per camera when there are several:
  - `left.pgm` and `right.pgm`;
  - `depth_mm.pgm` (16-bit);
  - the colour image as sent;
  - `cloud.ply`;
  - `info.txt`, with the calibration.
- **Several cameras**: the switches in the top bar pick which cameras are on screen. Every panel shows those cameras side by side.
  - GMSL with the host trigger shows the same trigger pulse in every tile. The Sync panel has the details.
  - PoE cameras each expose on their own clock.
- **Firewall**: the camera's streams come in through a firewall on this computer without a rule; the Link panel shows the camera port each stream comes from. Cameras with older firmware report none, and then a firewall drops the streams: the session opens but no data arrives, and the Control panel says so and how to allow the viewer.
- **PoE receive buffer**: the camera paces its output, so the system's default socket buffer is enough. The Link panel shows the buffer each stream got. If frames drop on a busy host, raise the limit with `sudo looper-cli setup-host`.
