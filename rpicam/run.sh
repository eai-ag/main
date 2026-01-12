#!/bin/bash

docker compose run --remove-orphans -it \
--volume ./ros2_ws/src/rpicam_gscam:/colcon_ws/src/rpicam_gscam \
rpicam bash

