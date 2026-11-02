# insight9P PoE 相机 快速启动指南

从开箱到在电脑上看到图像话题。全程约 15 分钟。

本文覆盖 **PoE 版本**（机身标识 `insight9P`）。

---

## 0. 准备：电脑侧

需要一台 Linux 电脑，装好 **ROS 2 Humble**（第 7 节订阅话题要用）。

安装 Insight SDK（含命令行工具 `looper-cli`），按电脑平台选择安装包：

| 平台 | 安装包 | 安装方式 |
|---|---|---|
| Ubuntu / Debian | `InsightSDK-1.0.0-linux-x86_64.deb` | `sudo apt install ./InsightSDK-1.0.0-linux-x86_64.deb`，安装到 `/usr` |
| 其他 Linux | `InsightSDK-1.0.0-linux-x86_64.tar.gz` | 解压到任意目录，`looper-cli` 在 `poe/bin/` |
| Windows x64 | `InsightSDK-1.0.0-windows-x64.zip` | 解压到任意目录，`looper-cli.exe` 和 `looper.dll` 在 `poe\bin\` |

确认版本：

```sh
looper-cli --version              # 输出 looper-cli 1.0.0 (Insight SDK)
```

本文后面直接写 `looper-cli`。用 tar.gz 或 zip 时，把它换成解压目录下的
`poe/bin/looper-cli`（Windows 为 `poe\bin\looper-cli.exe`）即可，其余参数完全一样。

---

## 1. 上电

用一根网线把相机接到 **PoE 交换机**或 PoE 注入器。相机没有独立电源口，供电和数据走同一根线。

> **不要同时插 USB 和网线。**
> 两个接口都能供电，**先插入的那个决定相机的工作模式**，之后插入的不会接管。
> 先插网线 = PoE 模式（本文场景），虚拟网卡 `usb0` 不会创建。
> 先插 USB = USB 模式，走另一套网络配置。
> 已经插错的话，拔掉全部接口，等 10 秒，按需要的顺序重新插。

上电后等约 1 分钟让相机启动完毕。

---

## 2. 相机的 IP 是怎么来的

不需要手动配置也能找到相机。开机后它按这个顺序自己决定地址：

| 顺序 | 行为 | 结果 |
|---|---|---|
| 1 | 请求 DHCP，**等 10 秒** | 拿到就用 DHCP 地址 |
| 2 | 10 秒没有 DHCP 服务器应答 | 回退到静态地址 **192.168.1.10** |
| 3 | 静态地址已被别人占用 | 回退到 169.254.x.x 链路本地地址 |

两个要点：

- **出厂默认 192.168.1.10 是回退值，不是固定值。** 网络里有 DHCP 服务器时相机会用 DHCP 地址。
- **第 3 种情况意味着两台相机撞车了。** 出厂默认地址相同，直接把两台接进同一个交换机，第二台会落到链路本地地址。按第 4 节给它们各自分配地址。

DHCP 只在**开机后的那 10 秒**内尝试。之后插入网线或启动 DHCP 服务器不会重新触发，需要重启相机。

---

## 3. 找到相机

用 SDK 自带的命令行工具。它走 UDP 广播，**即使相机 IP 和你的电脑不在同一网段也能找到**。

```sh
looper-cli discover --bind 192.168.1.1
```

```
MAC                 SERIAL       IP             MODE     CLOCK  NTP-SERVER    OFFSET
26:97:e8:a6:71:f6   0265400521   192.168.1.10   dhcp     yes    192.168.1.1   +0.1ms
6e:15:6c:fb:44:c6   0265400559   192.168.1.11   static   yes    192.168.1.1   -1.5ms
```

> **`--bind` 几乎总是需要写。**
> `--bind` 后面填**你电脑上连接相机那块网卡的 IP**（上例是有线网卡的 192.168.1.1，用 `ip -br addr` 查看）。
> 不写的话广播会从默认路由出去 —— 笔记本上通常是 WiFi，而相机在有线口上，结果就是"找不到相机"。

各列含义：

| 列 | 含义 |
|---|---|
| `SERIAL` | 板卡序列号，**认设备请用这个**（见下方提示） |
| `MODE` | `dhcp` / `static` / `link-local`。出现 `link-local` 说明地址撞车了 |
| `CLOCK` | 时间是否已同步。`no` 表示这台的时间戳不能和别的相机比对 |
| `NTP-SERVER` | 实际使用的时间源 |
| `OFFSET` | 最近一次校正量 |

> ⚠️ **当前固件 MAC 每次开机都会变。**
> 设备序列号 `SERIAL` 是稳定的，MAC 不是。第 4 节改 IP 是按 MAC 寻址的，所以
> **每次改 IP 前都要先跑一次 `discover` 拿当前 MAC**，不要记录 MAC 留着下次用。

---

## 4. 分配 IP 地址

多台相机接同一个交换机时必须做这一步，否则会撞车。

按 MAC 寻址，走广播，所以**相机 IP 不对、甚至不在同一网段也能改**。

```sh
# 永久静态地址（重启保留）—— 正式部署用这个
looper-cli set-static 26:97:e8:a6:71:f6 192.168.1.10 255.255.255.0 192.168.1.1
looper-cli set-static 6e:15:6c:fb:44:c6 192.168.1.11 255.255.255.0 192.168.1.1

# 临时地址（重启还原）—— 只是为了临时连上去，不是配置手段
looper-cli set-temp 26:97:e8:a6:71:f6 192.168.1.99

# 交还给 DHCP
looper-cli set-dhcp 26:97:e8:a6:71:f6
```

改完重新 `discover` 确认生效。地址跨网段变更后，相机可能需要几秒才重新出现。

---

## 5. 时间同步

**只要你要比对多台相机的时间戳，这一步就不能跳过。**

时间没同步时，两台相机的时间戳各自基于本机时钟，**不在同一基准上**，融合出来的结果是错的。`discover` 的 `CLOCK` 列就是用来确认这件事的。

出厂默认相机自动找时间源，顺序是：DHCP 下发的地址（option 42）→ 默认网关。小型网络里网关通常就能提供 NTP，开箱即用。

需要指定固定时间源时，用浏览器打开相机的配置页面（地址即相机 IP），在**网络设置**中填写
NTP 服务器地址，保存后立即生效。

> ⚠️ **填了就不再自动回退。**
> 一旦写入地址，相机只用这个地址。地址不可达时对时会直接失败（`CLOCK=no`），
> **不会**悄悄改用别的时间源。这是刻意设计的 —— 静默换源会让两台相机都显示
> "已同步"却各自对着不同的时钟。

> 📌 **参与时间戳比对的所有相机必须配同一个服务器。**
> 配了不同源的相机，每一台都会显示 `CLOCK=yes`，但相互之间对不齐。

**精度预期**：相机之间约 **1~3 毫秒**（实测中位数 0.92 ms，12000 组样本，范围 −1.58 ~ +5.92 ms）。
够用于图像级对齐（20~30 fps，帧间隔 33~50 ms）；**不足以**支撑 IMU 级紧耦合（400 Hz，间隔 2.5 ms）。

---

## 6. 【必做】先配 DDS 白名单，再订阅

**如果你的电脑有不止一块网卡（有线 + WiFi、或装了 VPN / Tailscale / Docker），
跳过这一步会让每台相机把同样的数据发 3 遍，白白吃掉 3 倍带宽。**

原因：Fast DDS 会把本机**每一块网卡**都宣告成一个接收地址，相机于是给每个地址各发一份完整拷贝。而这些拷贝全都从相机那唯一的网口挤出去，多出来的两份毫无用处。

实测（两台相机、默认话题全订阅）：

| 订阅端配置 | 相机 A | 相机 B | 合计 | 千兆占用 |
|---|---|---|---|---|
| 不限制（宣告 3 块网卡） | 445.2 Mbps | 433.7 Mbps | **878.9 Mbps** | 88% |
| 限制到 1 块网卡 | 147.4 Mbps | 144.7 Mbps | **292.1 Mbps** | 29% |

一条命令解决：

```sh
looper-cli dds-profile --bind 192.168.1.1 --out looper_fastdds_profile.xml
```

它会自动查出哪块网卡能到相机，并生成对应的配置文件。然后按它打印出来的绝对路径导出：

```sh
export FASTRTPS_DEFAULT_PROFILES_FILE=/abs/path/looper_fastdds_profile.xml
```

> **必须在启动订阅程序之前导出。** Fast DDS 只在创建参与者时读一次这个文件，
> 已经跑起来的进程不会生效。这也包括 `ros2` 命令行工具和 ROS 2 后台守护进程
> （先 `ros2 daemon stop`）。

如果命令提示 `net.core.rmem_max` 太小，一并执行它给出的命令：

```sh
sudo sysctl -w net.core.rmem_max=8388608
```

接收缓冲太小会让订阅端**在任何速率下**都丢约 0.3% 的数据包 —— 这是缓冲区溢出，不是带宽不够，100 Mbps 时和 900 Mbps 时一样丢。

---

## 7. 订阅话题

相机用 ROS 2 Humble 发布数据，`ROS_DOMAIN_ID=0`。电脑侧装好 ROS 2 Humble 即可。

```sh
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/abs/path/looper_fastdds_profile.xml   # 第 6 节

ros2 topic list
```

每台相机的话题在各自的命名空间下（如 `/camera_A/...`、`/camera_B/...`），多台相机因此不会互相覆盖。

单台相机的话题（实测速率，`insight9P`：中心 imx415 + 双目 sc132gs）：

| 话题（命名空间后部分） | 内容 | 带宽 | 频率 |
|---|---|---|---|
| `camera/infra1/image_rect_raw` | 左目灰度 640×544 mono8 | 7.02 MB/s | 20 Hz |
| `camera/infra2/image_rect_raw` | 右目灰度 640×544 mono8 | 6.97 MB/s | 20 Hz |
| `camera/color/image_rect_raw/compressed` | 彩色压缩图 | 3.01 MB/s | 30 Hz |
| `camera/imu` | IMU | 136.9 KB/s | 400 Hz |
| `camera/color/camera_info` | 彩色内参 | 11.3 KB/s | 30 Hz |
| `camera/vio_100hz` | VIO 位姿 | 8.5 KB/s | 100 Hz |
| `camera/vio_image` | VIO 可视化 | 1.7 KB/s | 20 Hz |
| `camera/vio_status` | VIO 状态 | 25 B/s | 1 Hz |

全部订阅时 **约 137 Mbps / 台**。

看一眼图像：

```sh
ros2 topic hz   /camera_A/camera/infra1/image_rect_raw
ros2 topic echo /camera_A/camera/vio_100hz
rqt_image_view /camera_A/camera/color/image_rect_raw/compressed
```

> `infra1/image_raw`、`infra2/image_raw`、`color/image_raw/compressed` 这几个话题
> 会出现在 `topic list` 里但**不发布数据**，带 `rect` 的才是实际在发的。

**没有订阅者时相机不发数据**（出口只有 0.03 Mbps 的发现流量）。所以带宽是按实际订阅产生的，不订阅就不占。

---

## 7a. 不用 ROS：用 SDK 直接取流

相机运行在**网口取流模式**时，不需要 ROS 和 DDS 白名单，SDK 直接收图像、深度、IMU 和 VIO 位姿。

```sh
looper-cli info   192.168.1.10                                  # 设备描述、内参、外参
looper-cli stream 192.168.1.10                                  # 全部 6 路，每秒打印帧率/码率/丢帧
looper-cli stream 192.168.1.10 --streams depth,imu,pose --seconds 10
```

```
ir_left     20 fps    56.2 Mbps  dropped 0  544x640 gray8
depth       11 fps    61.8 Mbps  dropped 0  544x640 z16
color       30 fps    14.8 Mbps  dropped 0  1920x1088 mjpeg
imu        401 fps     0.5 Mbps  dropped 0  acc [-0.007 -0.165 9.758] ...
pose        99 fps     0.1 Mbps  dropped 0  status 4 p [0.040 -0.092 0.073]
```

想直接看画面，用图形界面 **InsightViewer**（Linux 安装包 `InsightViewer-1.0.0-linux-x86_64.deb`，Windows 解压 `InsightViewer-1.0.0-windows-x64.zip`）：不带参数启动就会自动找到相机并打开，也能录制和截图。

写程序参考 `examples/3.stream.imu_pose`（回调收 IMU 和位姿）、`examples/4.stream.frameset`（左右目、深度、彩色按时间对齐成一组）、`examples/5.stream.viewer`（OpenCV 实时显示）。示例在 SDK 的 `poe/examples/` 目录（deb 安装在 `/usr/share/insightsdk/poe/examples/`）。接口说明见同目录的 [README.md](README.md) 的 Streaming 一节。

> 一台相机同一时间只接受**一个**取流主机，第二个会收到 `camera busy`。
> 主机一般不用做额外配置：相机发送有限速，系统默认的接收缓冲实测就不丢帧。
> 如果 `stream` 提示 `dropping frames with a ... receive buffer`（主机很忙或码率接近千兆时），
> 执行一次 `sudo looper-cli setup-host`，它会调大 `net.core.rmem_max` 并写入 `/etc/sysctl.d`，重启后仍然有效。
> 提示 `not in streaming mode`：相机还在 ROS 模式，按第 7 节用 ROS 订阅。

---

## 8. 带宽怎么算

相机的千兆口实测可跑满（TCP 940 Mbps 双向、零重传），所以瓶颈基本不在相机。规划时按这两条算：

1. **每台相机 ≈ 137 Mbps**（全话题，已做第 6 节的白名单）。
2. **每多一个订阅者，带宽线性翻倍。** 数据走单播，每个订阅端一份完整拷贝。实测单台相机：1 个订阅者 429.6 Mbps，2 个订阅者 859.3 Mbps（均为未做白名单的情况）。

所以一条千兆链路上：

| 场景 | 占用 |
|---|---|
| 2 台相机，1 个订阅端，已配白名单 | ~292 Mbps（29%） |
| 2 台相机，1 个订阅端，**未配**白名单 | ~879 Mbps（88%，接近打满） |
| 2 台相机，2 个订阅端，已配白名单 | ~584 Mbps（58%） |

多台相机共享交换机上行口时会**公平均分**（实测两台各 471 Mbps，合计 942 Mbps）。
只订阅需要的话题是最有效的减负手段。

---

## 9. 排查

| 现象 | 原因 / 处理 |
|---|---|
| `discover` 找不到相机 | 九成是没写 `--bind`，或写错了网卡 IP。用 `ip -br addr` 确认 |
| 相机 IP 是 169.254.x.x | 和另一台撞了出厂默认地址。按第 4 节分配不同地址 |
| 插了网线但相机没反应 | 先插的是 USB？两个接口互斥，拔掉全部等 10 秒再按顺序插 |
| 配了 DHCP 服务器但相机还是 192.168.1.10 | DHCP 只在开机后 10 秒内尝试，重启相机 |
| `ros2 topic list` 看得到话题，`echo` 没数据 | 名字带 `image_raw` 的空话题，用带 `rect` 的 |
| 带宽异常高 / 接近打满 | 没做第 6 节的白名单 |
| 丢帧、数据不连续 | `net.core.rmem_max` 太小，见第 6 节 |
| 改了白名单但带宽没降 | 订阅进程在导出环境变量之前就启动了。`ros2 daemon stop` 后重开 |
| `CLOCK` 显示 `no` | NTP 不可达。若手动配过 NTP 服务器，它不会自动兜底，检查地址是否可达 |
| 两台相机时间戳对不齐，但都显示 `CLOCK=yes` | 两台配了不同的 NTP 服务器，统一成同一个 |

以上步骤都试过仍未解决，请联系 Looper Robotics 技术支持，并提供：相机序列号
（`SERIAL`）、`looper-cli discover` 的完整输出、以及问题现象的描述。
