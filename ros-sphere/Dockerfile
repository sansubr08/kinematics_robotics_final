FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

# Install prerequisites for adding repositories
RUN apt-get update && apt-get install -y --no-install-recommends \
    lsb-release \
    gnupg \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Add the OSRF (Gazebo) repository
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# Install Gazebo Harmonic and ROS 2 dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-geometry-msgs \
    build-essential \
    python3-colcon-common-extensions \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ros2_ws
COPY src ./src

# Build ROS 2 workspace
RUN source /opt/ros/humble/setup.bash && colcon build

# Environment entrypoint setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch sphere_sim sphere_sim.launch.py"]