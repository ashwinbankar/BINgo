#!/bin/bash
# One-time setup for a fresh Raspberry Pi: install Docker, build both images, start both containers.
set -e
cd "$(dirname "$0")"

if ! command -v docker >/dev/null; then
  echo "Installing Docker..."
  curl -fsSL https://get.docker.com | sudo sh
  sudo usermod -aG docker "$USER"
fi

echo "Building the micro-ROS agent image (about 7 minutes)..."
sudo docker build -t bingo-lyrical-agent:auto .

echo "Building the LiDAR driver image (about 3 minutes)..."
sudo docker build -f Dockerfile.lidar -t bingo-lidar:auto .

echo "Starting the agent..."
./run_agent.sh

echo "Starting the LiDAR driver..."
./run_lidar.sh

echo "Done. Both containers restart by themselves on boot."
