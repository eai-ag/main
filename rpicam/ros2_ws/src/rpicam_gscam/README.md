# RPi Camera GStreamer ROS2 Package

This package provides a ROS2 launch file for the Raspberry Pi camera using gscam with dual output:
- ROS2 image topics (RGB8) via appsink
- TCP server for direct JPEG multipart streaming

## Camera Configuration

Fixed camera settings (matching docker-compose):
- Resolution: 1280x720
- Framerate: 30 fps
- Video crop: left=270, right=290
- TCP server: 0.0.0.0:5000

## Building

From the workspace root:

```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rpicam_gscam
source install/setup.bash
```

## Usage

Launch the camera node with default settings:

```bash
ros2 launch rpicam_gscam camera.launch.py
```

### Launch Arguments

You can customize the ROS2 topic namespace:

```bash
ros2 launch rpicam_gscam camera.launch.py \
    camera_name:=my_camera \
    frame_id:=my_camera_optical_frame
```

Available arguments:
- `camera_name` (default: `camera`) - Camera namespace for topics
- `frame_id` (default: `camera_optical_frame`) - TF frame ID

## Published Topics

- `/<camera_name>/image_raw` (sensor_msgs/Image) - Raw camera images
- `/<camera_name>/camera_info` (sensor_msgs/CameraInfo) - Camera calibration info

## TCP Streaming

The camera also streams JPEG images to a TCP server. Connect using:

```bash
# View stream with gst-launch
gst-launch-1.0 tcpclientsrc host=<hostname> port=5000 ! jpegdec ! autovideosink

# Or with ffplay
ffplay tcp://<hostname>:5000
```

## Pipeline Details

The GStreamer pipeline uses a `tee` element to split the stream:
1. One branch goes to `appsink` for ROS2 image publishing
2. Another branch goes to `jpegenc` -> `tcpserversink` for TCP streaming

This allows simultaneous ROS2 and TCP access to the camera stream.
