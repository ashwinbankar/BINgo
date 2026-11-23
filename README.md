# BINgo

Autonomous trash interceptor robot (MEEN 612, Texas A&M).

```
Laptop (vision + RViz2 dashboard) --Wi-Fi--> Raspberry Pi (micro-ROS agent + LiDAR, in Docker) --USB--> Teensy 4.1
```

## Repository Layout

| Folder      | Contents                                                      |
|-------------|---------------------------------------------------------------|
| `pi/`       | Dockerfiles, run scripts, `setup_pi.sh` (Pi side)             |
| `firmware/` | Teensy PlatformIO project                                     |
| `src/`      | ROS 2 packages: `bingo_vision`, `bingo_control`, `bingo_bringup` |
| `docs/`     | Engineering logs                                              |

## Daily Use (RViz2 Control Dashboard)

1. Power on the Pi with the Teensy and RPLIDAR plugged in. Wait 1-2 minutes for `bingo-agent` and `bingo-lidar` to auto-start.
2. On the laptop, launch the dashboard:

   ```bash
   source ~/BINgo/install/setup.bash
   ros2 launch bingo_bringup dashboard.launch.py
   ```

This opens RViz2 with `/scan` visualized and the docked **BingoControlPanel**:

- **Start LiDAR / Stop LiDAR**: calls `/start_motor` and `/stop_motor` to save battery when scanning isn't needed.
- **M0-M3 sliders** (`-1.0` to `1.0`): set individual motor speed and direction, published at 10 Hz on `/motor_cmd`.
- **STOP ALL MOTORS**: zeroes all sliders and cuts motor PWM immediately.

## Flash the Teensy (from the laptop)

1. Plug the Teensy into the laptop, then run:

   ```bash
   cd ~/BINgo/firmware && pio run -t upload
   ```

2. Press the white button on the Teensy when Teensy Loader asks.
3. Move the cable back to the Pi. It reconnects automatically within ~15 seconds.

## Set Up a Fresh Pi

```bash
scp -r ~/BINgo/pi <user>@<pi-ip>:~/bingo-pi
ssh <user>@<pi-ip> "cd ~/bingo-pi && ./setup_pi.sh"
```

> Not yet tested on a blank Pi.

## Troubleshooting

- Agent log (on the Pi): `sudo docker logs -f bingo-agent`
- LiDAR log (on the Pi): `sudo docker logs -f bingo-lidar`
- ROS domain must be `0` everywhere.
- Teensy not connecting: check the agent is running (`sudo docker ps`), then wait 15 seconds.
- Don't press the Teensy button to reset it, because it enters bootloader mode. Replug the USB cable instead.

## LiDAR (RPLIDAR A1)

Plug the LiDAR's USB adapter into a Pi USB port. The driver runs in Docker (`bingo-lidar`), starts on boot, and publishes `/scan` (~6 Hz over Wi-Fi).

CLI checks and motor standby controls (on the laptop):

```bash
ros2 topic hz /scan
ros2 service call /stop_motor std_srvs/srv/Empty
ros2 service call /start_motor std_srvs/srv/Empty
```

Notes:

- Use a 5V 3A power supply (or buck converter) for the Pi. A weak supply causes `"Can not start scan"` in the driver log. Check with `vcgencmd get_throttled` (you want `0x0`).
- Rebuild the image (on the Pi):

  ```bash
  cd ~/bingo-pi && sudo docker build -f Dockerfile.lidar -t bingo-lidar:auto . && ./run_lidar.sh
  ```

## Motor Control (4-Motor Mecanum Bench Bring-Up)

The Teensy 4.1 node (`teensy_motor_node`) subscribes over micro-ROS (`ROS_DOMAIN_ID=0`) to:

- `/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/motor_cmd` (`std_msgs/msg/Float32MultiArray`)

It drives four FIT0186 DC motors through two Cytron MDD10A drivers with a 500 ms command-timeout watchdog.

### Active Pin Mapping & Calibration (Bench Firmware Override)

> **Wiring Note:** On the current bench harness, the `PWM` and `DIR` jumpers are swapped relative to Schematic Rev. 2.0, and the motor channels run clockwise around the chassis (`RR -> RL -> FL -> FR`). Both are compensated in `firmware/src/main.cpp` (`DIR_PINS`/`PWM_PINS` swapped, `MOTOR_POLARITY = {-1, 1, 1, -1}`, and updated `cmd_vel_callback` mixing) until the lab rewiring overhaul.

| Motor | Physical Corner | Driver | DIR pin (FW) | PWM pin (FW) | Polarity |
|-------|-----------------|--------|--------------|--------------|----------|
| M0    | Rear-Right      | D0     | 9            | 8            | `-1`     |
| M1    | Rear-Left       | D0     | 11           | 10           | `1`      |
| M2    | Front-Left      | D1     | 24           | 12           | `1`      |
| M3    | Front-Right     | D1     | 28           | 25           | `-1`     |

### CLI Motor Tests & Teleop

Publish at 10 Hz to keep the 500 ms watchdog active:

```bash
# Holonomic keyboard teleop (repeat_rate keeps watchdog fed)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=10.0 -p speed:=0.4 -p turn:=0.4

# Mecanum twist command (all 4 motors)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"

# Individual motor override [M0, M1, M2, M3]
ros2 topic pub -r 10 /motor_cmd std_msgs/msg/Float32MultiArray "{data: [0.4, 0.0, 0.0, 0.0]}"
