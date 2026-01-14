#!/bin/bash

docker compose run --remove-orphans -it  \
-v $(pwd)/camera.launch.py:/root/ros2_humble/install/gscam/share/gscam/examples/camera.launch.py \
rpicam bash

