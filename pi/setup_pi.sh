#!/bin/bash
# One-time setup for a fresh Raspberry Pi: install Docker, build the agent, start it.
set -e
cd "$(dirname "$0")"

if ! command -v docker >/dev/null; then
  echo "Installing Docker..."
  curl -fsSL https://get.docker.com | sudo sh
  sudo usermod -aG docker "$USER"
fi

echo "Building the agent image (about 7 minutes)..."
sudo docker build -t bingo-lyrical-agent:auto .

echo "Starting the agent..."
./run_agent.sh

echo "Done. Unplug and replug nothing: the Teensy will connect by itself."
