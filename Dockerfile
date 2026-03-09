FROM osrf/ros:humble-desktop-full

# Set up environment
ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-teleop \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-msgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-xacro \
    xterm \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /root/ros2_ws
RUN mkdir -p src

# Source ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

# Default command
CMD ["bash"]
