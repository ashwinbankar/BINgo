FROM ros:lyrical-ros-base
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    git build-essential python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /microros_ws
RUN mkdir src && cd src \
    && git clone -b rolling https://github.com/micro-ROS/micro-ROS-Agent.git \
    && git clone -b rolling https://github.com/micro-ROS/micro_ros_msgs.git

RUN source /opt/ros/lyrical/setup.bash \
    && (rosdep init || true) && rosdep update \
    && rosdep install --from-paths src --ignore-src -y --skip-keys "microxrcedds_agent microxrcedds_client"

RUN source /opt/ros/lyrical/setup.bash \
    && colcon build --packages-up-to micro_ros_agent \
       --cmake-args -DCMAKE_BUILD_TYPE=Release \
       -DUAGENT_DISCOVERY_PROFILE=OFF -DUAGENT_P2P_PROFILE=OFF

ENV ROS_DOMAIN_ID=0
CMD ["bash", "-c", "source /opt/ros/lyrical/setup.bash && source /microros_ws/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200"]
