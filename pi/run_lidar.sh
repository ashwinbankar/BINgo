#!/bin/bash
# Start the BINgo LiDAR driver
sudo true

sudo docker rm -f bingo-lidar 2>/dev/null

sudo docker run -d --name bingo-lidar \
  --network host \
  --restart unless-stopped \
  --device=/dev/ttyUSB0 \
  -e ROS_DOMAIN_ID=0 \
  bingo-lidar:auto \
  bash -c "source /opt/ros/lyrical/setup.bash && source /lidar_ws/install/setup.bash && ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser"

echo "LiDAR started. Watch it with: sudo docker logs -f bingo-lidar"
