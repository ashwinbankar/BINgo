# BINgo

Autonomous trash interceptor robot (MEEN 612, Texas A&M).
Laptop (vision) -> Wi-Fi -> Raspberry Pi (micro-ROS agent in Docker) -> USB -> Teensy 4.1.

## Daily use

1. Power on the Pi with the Teensy plugged in. Wait 1-2 minutes. The agent starts by itself.
2. On the laptop, open a new terminal and test:

    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"   # LED ON
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"   # LED OFF

## Folders

- `pi/`       Dockerfile, run_agent.sh, setup_pi.sh (Pi side)
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
