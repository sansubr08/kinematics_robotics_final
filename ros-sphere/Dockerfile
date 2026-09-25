FROM tiryoh/ros2-desktop-vnc:jazzy

SHELL ["/bin/bash", "-c"]

# 1. Install prerequisites, Gazebo repositories, and NVIDIA hardware layers in ONE pass
RUN apt-get update && apt-get install -y --no-install-recommends \
    lsb-release \
    gnupg \
    curl \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-geometry-msgs \
    build-essential \
    python3-colcon-common-extensions \
    gz-harmonic \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libgles2 \
    libxcb-glx0 \
    && rm -rf /var/lib/apt/lists/*

# 2. Strict Hardware Pass-through Configuration
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all,graphics,utility,display

# Force GLVND to locate your physical NVIDIA card over Mesa
ENV FORCE_FLUENT_LOGGING=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia

# Set up workspace
WORKDIR /ros2_ws
COPY src ./src

# Build ROS 2 workspace
RUN source /opt/ros/jazzy/setup.bash && colcon build

# Environment entrypoint setup
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch sphere_sim sphere_sim.launch.py"]
