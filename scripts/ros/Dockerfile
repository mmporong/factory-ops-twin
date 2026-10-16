FROM ros:jazzy-ros-base
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-rosbag2 \
    ros-jazzy-rmw-fastrtps-cpp python3-yaml \
    && rm -rf /var/lib/apt/lists/*
ENV ROS_DOMAIN_ID=71 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
WORKDIR /workspace/project
