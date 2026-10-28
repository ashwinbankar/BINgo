# BINgo

Vision-based mobile trash interceptor robot for MEEN 612.

## Raspberry Pi Environment

The Raspberry Pi runs Debian, so we use Docker to run the Ubuntu-based ROS 2 Lyrical environment natively over the host network.

Start the interactive container on the Pi:

```bash
sudo docker run -it --network host --rm ros:lyrical-ros-base /bin/bash
```

Once inside the container, set the domain ID and activate ROS 2:

```bash
export ROS_DOMAIN_ID=10
source /opt/ros/lyrical/setup.bash
```

## iPhone Camera & Stream Setup

1. Unlock your phone and open the SensorStream app.
2. Set **Depth Map** OFF and **Rear Camera** ON, then plug the phone into your laptop via USB.
3. Clear the USB port and launch the ROS 2 driver (Terminal 1):

   ```bash
   killall gvfs-afc-volume-monitor gvfs-gphoto2-volume-monitor
   sudo systemctl restart usbmuxd
   source /opt/ros/lyrical/setup.bash
   source ~/BINgo/install/setup.bash
   ros2 run sensorstream_driver sensorstream_node
   ```

4. Tap **Start** on the iPhone app.

## Visualization & Object Tracking

### View in RViz2 (Terminal 2)

```bash
source /opt/ros/lyrical/setup.bash
source ~/BINgo/install/setup.bash
rviz2
```

- Set the **Fixed Frame** to `color_image`.
- Add an **Image** display for `/color_image/compressed`.
- Change the **Reliability Policy** to `Best Effort`.

### Run YOLOv8 Ball Tracker (Terminal 3)

```bash
source /opt/ros/lyrical/setup.bash
source ~/BINgo/install/setup.bash
cd ~/BINgo
python3 src/vision/src/ball_tracker.py
```
