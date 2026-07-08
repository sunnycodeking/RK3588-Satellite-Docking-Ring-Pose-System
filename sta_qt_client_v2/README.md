# Satellite Qt Client V2

本版本解决三个问题：

1. 日志区 JSON 引号显示为 `&quot;` 的问题：改为 `QTextCursor::insertText()` 纯文本插入，不再把普通日志当 HTML 处理。
2. RK3588 检测程序调试信息不能显示的问题：提供新的 RK3588 TCP Server，使用 pipe 捕获检测子进程 stdout/stderr，并通过 `{"type":"LOG"}` JSON 消息发回 Qt。
3. 视频流嵌入 Qt 页面内部：`VideoReceiver` 改为 GStreamer appsink 方式接收 RTP/JPEG，将帧转换为 `QImage` 后显示在 Dashboard 页面内，不再使用 `autovideosink` 弹出独立窗口。

## Ubuntu 编译依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config \
    qtbase5-dev qtbase5-dev-tools \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-base
```

如果使用 Qt6，请安装对应 Qt6 Widgets/Network 开发包。

## 编译 Qt 客户端

```bash
cd satellite_qt_client_v2
mkdir build
cd build
cmake ..
make -j$(nproc)
./SatelliteQtClientV2
```

也可以：

```bash
qmake SatelliteQtClientV2.pro
make -j$(nproc)
./SatelliteQtClientV2
```

## RK3588 服务端编译

将 `server/rk3588_detect_track_server_with_log.c` 拷贝到 RK3588，编译：

```bash
gcc rk3588_detect_track_server_with_log.c -o rk3588_detect_track_server_with_log -Wall -O2
./rk3588_detect_track_server_with_log 8888
```

它会在收到 Qt 的 `START_MODE detect_track` 后执行：

```bash
cd /hzy_sta/rknn_yolov8_camera_demo
./rknn_yolov8_camera_demo ./model/yolov8.rknn
```

## 调试信息说明

服务器通过 pipe 捕获检测程序 stdout/stderr。若检测程序 printf 后不换行或没有 fflush，调试信息可能延迟显示。若你能修改检测程序源码，建议在 main() 开头加入：

```c
setvbuf(stdout, NULL, _IOLBF, 0);
setvbuf(stderr, NULL, _IONBF, 0);
```
