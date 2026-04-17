# ROS 2 Jazzy Autonomous Explorer Container
FROM osrf/ros:jazzy-desktop AS base

SHELL ["/bin/bash", "-c"]

# Install Nav2, Gazebo, SLAM Toolbox, and a basic VNC server for GUI access
RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz \
    python3-colcon-common-extensions \
    python3-pip \
    vim \
    tigervnc-standalone-server \
    tigervnc-common \
    dbus-x11 \
    x11-xserver-utils \
    xfce4 \
    xfce4-terminal \
    && rm -rf /var/lib/apt/lists/*

ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=/opt/ros/jazzy/share/turtlebot3_gazebo/models
ENV DISPLAY=:1
ENV VNC_RESOLUTION=1920x1080
ENV VNC_PORT=5901

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

# Development stage — workspace is mounted, not copied
FROM base AS dev

WORKDIR /workspaces/ros2_ws

RUN mkdir -p /root/.vnc && \
    printf '#!/bin/bash\nunset SESSION_MANAGER\nunset DBUS_SESSION_BUS_ADDRESS\nexec startxfce4\n' > /root/.vnc/xstartup && \
    chmod +x /root/.vnc/xstartup && \
    printf 'password\npassword\nn\n' | vncpasswd

RUN echo 'if [ -f /workspaces/ros2_ws/install/setup.bash ]; then source /workspaces/ros2_ws/install/setup.bash; fi' >> /etc/bash.bashrc

CMD ["bash"]

# Production stage — copies and builds the workspace into the image
FROM base AS production

WORKDIR /ros2_ws
COPY ./src /ros2_ws/src
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install
RUN echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc

CMD ["bash"]
