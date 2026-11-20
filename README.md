# BINgo

Autonomous trash interceptor robot (MEEN 612, Texas A&M).
Laptop (vision) -> Wi-Fi -> Raspberry Pi (micro-ROS agent in Docker) -> USB -> Teensy 4.1.

## Daily use

1. Power on the Pi with the Teensy plugged in. Wait 1-2 minutes. The agent starts by itself.
2. On the laptop, open a new terminal and test:

    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"   # LED ON
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"   # LED OFF

## Folders

- `pi/`       Dockerfiles, run scripts, setup_pi.sh (Pi side)
- `firmware/` Teensy PlatformIO project
- `src/`      ROS 2 packages (vision, control, bringup)
- `docs/`     Engineering logs

## Flash the Teensy (from the laptop)

Plug the Teensy into the laptop, then:

    cd ~/BINgo/firmware && pio run -t upload

Press the white button on the Teensy when Teensy Loader asks. Move the cable back to the Pi.

## Set up a fresh Pi

    scp -r ~/BINgo/pi <user>@<pi-ip>:~/bingo-pi
    ssh <user>@<pi-ip> "cd ~/bingo-pi && ./setup_pi.sh"

(Not yet tested on a blank Pi.)

## Troubleshooting

- Agent log (on the Pi): `sudo docker logs -f bingo-agent`
- ROS domain must be 0 everywhere.
- Teensy not connecting: check the agent is running (`sudo docker ps`), then wait 15 seconds.
- Don't press the Teensy button to reset it. It enters bootloader mode. Replug the USB cable instead.

## LiDAR (RPLIDAR A1)

Plug the LiDAR's USB adapter into a Pi USB port. The driver runs in Docker (`bingo-lidar`), starts on boot, and publishes `/scan`.

Check on the laptop:

    ros2 topic hz /scan        # about 6 Hz over Wi-Fi

View it in RViz:

    rviz2

Set Fixed Frame to `laser`, then Add -> By topic -> `/scan` -> LaserScan. If you see nothing, set Reliability Policy to Best Effort.

Notes:

- Use a 5V 3A power supply for the Pi. A weak supply gives "Can not start scan" in the driver log. Check with `vcgencmd get_throttled` (want `0x0`).
- Driver log (on the Pi): `sudo docker logs -f bingo-lidar`
- Rebuild the image (on the Pi): `cd ~/bingo-pi && sudo docker build -f Dockerfile.lidar -t bingo-lidar:auto . && ./run_lidar.sh`

## Motor Control (4-Motor Mecanum Bench Bring-Up)

The Teensy 4.1 node (`teensy_motor_node`) subscribes to `/cmd_vel` (`geometry_msgs/msg/Twist`) over micro-ROS (`ROS_DOMAIN_ID=0`) and drives four FIT0186 DC motors through two Cytron MDD10A drivers (`D0` = Left, `D1` = Right) with a 500 ms command timeout watchdog.

### Pin Mapping (Schematic Rev. 2.0)
- **M0 (Front-Left, D0):** `DIR = Pin 8`, `PWM = Pin 9`
- **M1 (Rear-Left, D0):** `DIR = Pin 10`, `PWM = Pin 11`
- **M2 (Front-Right, D1):** `DIR = Pin 12`, `PWM = Pin 24`
- **M3 (Rear-Right, D1):** `DIR = Pin 25`, `PWM = Pin 28`

### Running a Motor Test
Publish at 10 Hz from the workstation to keep the 500 ms watchdog active (press `Ctrl+C` to stop all motors):
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"
