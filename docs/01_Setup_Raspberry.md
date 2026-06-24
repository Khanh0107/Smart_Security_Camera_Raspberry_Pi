# 01_Setup_Raspberry.md

# Project: RTSP Camera Streaming via MediaMTX + Smart Recording + Face Detection

## Objective

Build a smart camera system on Raspberry Pi 4 using:

* Raspberry Pi Camera Module 3 (IMX708)
* GStreamer + libcamera + V4L2 hardware H.264 encoder
* OpenCV (Haar cascade face detection)
* C++ (multithreaded: GStreamer threads + OpenCV worker thread)
* MediaMTX (standalone RTSP media server the app publishes to)
* RTSP Streaming (viewable from VLC / smartphone, including remotely via Tailscale)
* Smart recording: only saves video while a face is detected

---

# Project Architecture

```text
Camera Module 3 (IMX708)
        │
        ▼
   libcamerasrc
        │
        ▼
videoconvert + overlays (logo, clock)
        │
        ▼
     appsink ──────────────► [OpenCV worker thread]
                                    │  face detect (Haar cascade)
                                    │  draw bounding box
                                    ▼
                                appsrc
                                    │
                                    ▼
                              videoconvert
                                    │
                              v4l2h264enc
                          (single shared encoder)
                                    │
                              h264parse
                                    │
                                   tee
                                 ╱     ╲
                  Publish branch         Recording branch
                 rtspclientsink           valve (on/off)
                 (publish to MediaMTX)    h264parse
                        │                 splitmuxsink → .mp4
                        ▼
                  MediaMTX server
                  (rtsp://<PI_IP>:8554/camera)
                        │
                        ▼
              VLC / phone (LAN or via Tailscale)
```

The app does **not** host its own RTSP server. It pushes ("publishes") an encoded H.264 stream into a separately running MediaMTX process via `rtspclientsink`, and MediaMTX serves that stream to any number of RTSP clients (VLC, phones, etc.).

---

# Hardware Requirements

## Raspberry Pi

* Raspberry Pi 4 Model B
* 4GB RAM or 8GB RAM recommended

## Camera

* Raspberry Pi Camera Module 3 (Sony IMX708 sensor)
* CSI ribbon cable connected securely on both ends

## Display (Optional, for local debugging)

* HDMI Monitor

---

# Operating System

Recommended:

```text
Raspberry Pi OS Bookworm (64-bit)
```

Check current OS version:

```bash
cat /etc/os-release
```

---

# Step 1: Update the System

```bash
sudo apt update
sudo apt upgrade -y
```

---

# Step 2: Install Development Tools

```bash
sudo apt install -y \
build-essential \
cmake \
git \
pkg-config
```

Verify installation:

```bash
gcc --version
g++ --version
cmake --version
```

---

# Step 3: Install GStreamer Core

```bash
sudo apt install -y \
gstreamer1.0-tools \
libgstreamer1.0-dev \
libgstreamer-plugins-base1.0-dev
```

---

# Step 4: Install GStreamer Plugins

```bash
sudo apt install -y \
gstreamer1.0-plugins-base \
gstreamer1.0-plugins-good \
gstreamer1.0-plugins-bad \
gstreamer1.0-plugins-ugly \
gstreamer1.0-libav
```

`gstreamer1.0-plugins-good` is required for `v4l2h264enc` (hardware H.264 encoder used in `main.cpp`).

`gstreamer1.0-plugins-bad` is required for `rtspclientsink`, the element `main.cpp` uses to **publish** the stream into MediaMTX.

---

# Step 5: Install libcamera GStreamer Plugin

```bash
sudo apt install -y gstreamer1.0-libcamera
```

### Why this package matters

It provides the GStreamer source element used at the top of the pipeline in `main.cpp`:

```text
libcamerasrc
```

Quick test:

```bash
gst-launch-1.0 libcamerasrc ! autovideosink
```

Verify installation:

```bash
gst-inspect-1.0 libcamerasrc
```

---

# Step 6: Install GStreamer App Library (appsink / appsrc)

`main.cpp` bridges GStreamer and OpenCV using `appsink` (camera → OpenCV) and `appsrc` (OpenCV → encoder). This requires:

```bash
sudo apt install -y \
gstreamer1.0-plugins-base \
libgstreamer-plugins-base1.0-dev
```

Verify `gstreamer-app-1.0` module is available:

```bash
pkg-config --modversion gstreamer-app-1.0
```

---

# Step 7: Verify rtspclientsink (RTSP publish element)

Unlike earlier versions of this project, `main.cpp` no longer hosts its own `GstRTSPServer`. Instead it **publishes** to MediaMTX using `rtspclientsink`, provided by `gstreamer1.0-plugins-bad` (already installed in Step 4).

Verify it's available:

```bash
gst-inspect-1.0 rtspclientsink
```

If missing:

```bash
sudo apt install --reinstall gstreamer1.0-plugins-bad
```

---

# Step 8: Install OpenCV

```bash
sudo apt install -y \
libopencv-dev \
opencv-data
```

`opencv-data` provides the Haar cascade XML files used in `main.cpp`:

```text
/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml
/usr/share/opencv4/haarcascades/haarcascade_profileface.xml
```

Verify OpenCV version:

```bash
pkg-config --modversion opencv4
```

Verify cascade files exist:

```bash
ls /usr/share/opencv4/haarcascades/ | grep -E "frontalface_default|profileface"
```

---

# Step 9: Install libcamera Tools

```bash
sudo apt install -y \
libcamera-tools \
libcamera-dev
```

Verify camera operation:

```bash
rpicam-hello -t 5000
```

---

# Step 10: Verify Camera Functionality

Capture a JPEG image:

```bash
rpicam-jpeg -o test.jpg
```

Record a short video:

```bash
rpicam-vid -t 5000 -o test.h264
```

---

# Step 11: Verify GStreamer Installation

Check version:

```bash
gst-launch-1.0 --version
```

Run a simple test pipeline:

```bash
gst-launch-1.0 videotestsrc ! autovideosink
```

---

# Step 12: Verify libcamerasrc + GStreamer Integration

```bash
gst-launch-1.0 libcamerasrc ! videoconvert ! autovideosink
```

If a live camera image appears:

```text
Camera + GStreamer Integration is Working
```

---

# Step 13: Verify Hardware H.264 Encoder (v4l2h264enc)

`main.cpp` uses the Pi's VideoCore hardware encoder instead of software `x264enc` to keep CPU usage low. A single encoder instance feeds both the publish branch and the recording branch via `tee`.

```bash
gst-inspect-1.0 v4l2h264enc
```

Check the encoder device exists:

```bash
ls /dev/video*
```

If `v4l2h264enc` is missing, reinstall good plugins:

```bash
sudo apt install --reinstall gstreamer1.0-plugins-good
```

---

# Step 14: Install Git

```bash
sudo apt install -y git
```

Verify:

```bash
git --version
```

---

# Step 15: Install MediaMTX

MediaMTX is a standalone RTSP media server. It runs as its own process, separate from this repository, and is what RTSP clients (VLC, phones) actually connect to.

```bash
cd ~
wget https://github.com/bluenviron/mediamtx/releases/download/v1.12.3/mediamtx_v1.12.3_linux_arm64.tar.gz
mkdir mediamtx
tar xzvf mediamtx_v1.12.3_linux_arm64.tar.gz -C mediamtx
cd mediamtx
```

Check the [MediaMTX releases page](https://github.com/bluenviron/mediamtx/releases) for the latest version; use the `arm64` variant for 64-bit Raspberry Pi OS.

Edit `mediamtx.yml` and configure the `camera` path to accept a publisher:

```bash
nano mediamtx.yml
```

Save and exit (Ctrl+O, Enter, Ctrl+X).

Test that MediaMTX starts correctly:

```bash
./mediamtx
```

Expected output includes:

```text
INF [RTSP] listener opened on :8554 (TCP), :8000 (UDP/RTP), :8001 (UDP/RTCP)
```

Leave this running in its own terminal — `main.cpp` will fail to publish if MediaMTX is not running.

---

# Step 16: Create Project Workspace

```bash
mkdir -p ~/rtsp_camera/{src,build,logo,recordings}
cd ~/rtsp_camera
```

Place your penguin logo (or any PNG/SVG overlay) inside `logo/`:

```bash
cp /path/to/penguin-svgrepo.svg ~/rtsp_camera/logo/
```

---

# Project Directory Structure

```text
rtsp_camera/

├── CMakeLists.txt
│
├── src/
│   └── main.cpp
│
├── logo/
│   └── penguin-svgrepo.svg
│
├── recordings/
│   └── detect.log          (created automatically at runtime)
│
└── build/

mediamtx/                   (installed separately, outside this repo)
├── mediamtx
└── mediamtx.yml
```

---

# Step 17: CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.10)
project(rtsp_dual)

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

pkg_check_modules(GST  REQUIRED gstreamer-1.0)
pkg_check_modules(APP  REQUIRED gstreamer-app-1.0)

add_executable(app src/main.cpp)

target_include_directories(app PRIVATE
    ${GST_INCLUDE_DIRS}
    ${APP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(app
    ${GST_LIBRARIES}
    ${APP_LIBRARIES}
    ${OpenCV_LIBS}
)
```

`gstreamer-rtsp-server-1.0` is **no longer linked** — the app does not host its own RTSP server. `rtspclientsink` (used to publish to MediaMTX) is part of `gstreamer-1.0` plugins resolved at runtime via `gst_parse_launch`, not a separate pkg-config module.

---

# Step 18: Build the Project

```bash
cd ~/rtsp_camera/build
cmake ..
make -j4
```

---

# Step 19: Run

Make sure MediaMTX is already running (Step 15) in its own terminal before starting the app.

```bash
cd ~/rtsp_camera/build
./app
```

Expected console output includes:

```text
[HW] v4l2h264enc OK
[GST] rtspclientsink OK
[OpenCV] Frontal cascade loaded OK
[OpenCV] Profile cascade loaded OK
[appsrc] Ready
[Recorder] Callback registered
[Valve] Ready (drop=true, recording paused)
[Pipeline] Publishing to MediaMTX started
```

View the stream from VLC or a phone:

```text
rtsp://<PI_IP>:8554/camera
```

Find the Pi's IP address:

```bash
hostname -I
```

---

# Step 20 (Optional): Set Correct Timezone

Ensure timestamps in the video overlay and log file match local time:

```bash
sudo timedatectl set-timezone Asia/Ho_Chi_Minh
timedatectl
```

---

# Step 21 (Optional): Remote Viewing via Tailscale

To view the stream from outside the local network (mobile data, a different Wi-Fi), install Tailscale on the Pi and on the viewing device.

On the Pi:

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```

Open the printed login link in a browser and authenticate. Then get the Pi's Tailscale IP:

```bash
tailscale ip -4
```

Install the Tailscale app on the phone, sign in with the same account, and enable the VPN toggle. Connect VLC using the Tailscale IP instead of the LAN IP:

```text
rtsp://100.x.x.x:8554/camera
```

Check both devices are online:

```bash
tailscale status
```

---

# Expected Result

After completing this setup:

* Camera is working with `libcamerasrc`
* GStreamer core, plugins (including `plugins-bad` for `rtspclientsink`), libcamera plugin, and app library are installed
* Hardware H.264 encoder (`v4l2h264enc`) is available
* MediaMTX is installed and running, listening on port 8554
* OpenCV with Haar cascade files is installed
* `main.cpp` builds successfully via CMake
* The app publishes its stream into MediaMTX, viewable from VLC / smartphone
* Recording only triggers when a face is detected, saved to `recordings/`
* Detection events are logged to `recordings/detect.log`
* (Optional) Stream is viewable remotely via Tailscale

---

# Final Verification Checklist

Verify libcamerasrc:

```bash
gst-inspect-1.0 libcamerasrc
```

Verify camera capture:

```bash
rpicam-jpeg -o test.jpg
```

Verify GStreamer:

```bash
gst-launch-1.0 --version
```

Verify appsink/appsrc library:

```bash
pkg-config --modversion gstreamer-app-1.0
```

Verify rtspclientsink:

```bash
gst-inspect-1.0 rtspclientsink
```

Verify hardware encoder:

```bash
gst-inspect-1.0 v4l2h264enc
```

Verify OpenCV + cascades:

```bash
pkg-config --modversion opencv4
ls /usr/share/opencv4/haarcascades/ | grep frontalface_default
```

Verify MediaMTX is running and listening:

```bash
ss -tlnp | grep 8554
```

If all commands succeed:

```text
Environment Ready — proceed to build and run main.cpp
```

---

# Quick Install (All-in-One)

For a fresh Raspberry Pi OS Bookworm install, run all GStreamer/OpenCV dependencies in one block:

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install -y \
  build-essential cmake git pkg-config \
  gstreamer1.0-tools \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-libcamera \
  libcamera-tools \
  libcamera-dev \
  libopencv-dev \
  opencv-data
```

Then install MediaMTX separately (see Step 15) — it is a standalone binary, not an apt package.