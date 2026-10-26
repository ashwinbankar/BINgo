# BINgo

Vision-based mobile trash interceptor robot for MEEN 612

## Raspberry Pi Environment

The Raspberry Pi runs Debian, so we use Docker to run the Ubuntu-based ROS 2 Lyrical environment natively over the host network.

To start the interactive container on the Pi, run:

```bash
sudo docker run -it --network host --rm ros:lyrical-ros-base /bin/bash
```

Once inside the container, set the domain ID and activate ROS 2:

```bash
export ROS_DOMAIN_ID=10
source /opt/ros/lyrical/setup.bash
```
