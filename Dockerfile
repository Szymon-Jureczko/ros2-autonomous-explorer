# ROS 2 Jazzy Autonomous Explorer Container
# Supports dev container with VNC for GUI (Gazebo, RViz)

FROM osrf/ros:jazzy-desktop AS base

SHELL ["/bin/bash", "-c"]

# Install system dependencies (Nav2, Gazebo, and VNC)
RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz \
    python3-colcon-common-extensions \
    python3-pip \
    vim \
    # VNC and desktop dependencies
    tigervnc-standalone-server \
    tigervnc-common \
    novnc \
    websockify \
    dbus-x11 \
    x11-xserver-utils \
    xfce4 \
    xfce4-terminal \
    && rm -rf /var/lib/apt/lists/*

# Set up environment
ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=/opt/ros/jazzy/share/turtlebot3_gazebo/models
ENV DISPLAY=:1
ENV VNC_RESOLUTION=1920x1080
ENV VNC_PORT=5901
ENV NOVNC_PORT=6080

# Source ROS in all bash shells
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

# ============================================
# Development stage - for VS Code dev container
# Workspace is mounted, not copied
# ============================================
FROM base AS dev

WORKDIR /workspaces/ros2_ws

# Create VNC startup script and set password non-interactively
RUN mkdir -p /root/.vnc && \
    printf '#!/bin/bash\nunset SESSION_MANAGER\nunset DBUS_SESSION_BUS_ADDRESS\nexec startxfce4\n' > /root/.vnc/xstartup && \
    chmod +x /root/.vnc/xstartup && \
    printf 'password\npassword\nn\n' | vncpasswd

# Create script to start VNC and noVNC
COPY <<'EOF' /usr/local/bin/start-vnc.sh
#!/bin/bash
# Kill any existing VNC sessions
vncserver -kill :1 2>/dev/null || true

# Start VNC server
vncserver :1 -geometry ${VNC_RESOLUTION} -depth 24 -localhost no

# Start noVNC (web-based VNC client)
/usr/share/novnc/utils/novnc_proxy --vnc localhost:${VNC_PORT} --listen ${NOVNC_PORT} &

echo "============================================"
echo "VNC server started on port ${VNC_PORT}"
echo "noVNC web client: http://localhost:${NOVNC_PORT}/vnc.html"
echo "VNC password: password"
echo "============================================"
EOF
RUN chmod +x /usr/local/bin/start-vnc.sh

# Source workspace setup if it exists
RUN echo 'if [ -f /workspaces/ros2_ws/install/setup.bash ]; then source /workspaces/ros2_ws/install/setup.bash; fi' >> /etc/bash.bashrc

CMD ["bash"]

# ============================================
# Production stage - standalone deployment
# Copies and builds the workspace into image
# ============================================
FROM base AS production

WORKDIR /ros2_ws

# Copy source and build
COPY ./src /ros2_ws/src
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install

# Source workspace
RUN echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc

CMD ["bash"]
