#!/bin/bash
# Start the BINgo micro-ROS agent
sudo true   # ask for the password first, before the container starts

sudo docker rm -f bingo-agent 2>/dev/null

sudo docker run -d --name bingo-agent \
  --network host \
  --restart unless-stopped \
  --device=/dev/ttyACM0 \
  -e ROS_DOMAIN_ID=0 \
  bingo-lyrical-agent:auto

echo "Agent started. Watch it with: sudo docker logs -f bingo-agent"
