# Full simulation image (x86 workstation only):
#   ArduPilot SITL Rover 4.7.2 (same firmware version as the truck's FC)
#   + Gazebo Harmonic physics/cameras (ardupilot_gazebo JSON interface)
#   + ROS2 Humble: ros_gz bridge, stereo_image_proc, RTAB-Map, rviz2, teleop
#   + a browser desktop (noVNC on http://localhost:6080) for the Gazebo/RViz GUIs,
#     because Docker Desktop can't share the host's X display or GPU.
#
# Used by docker/docker-compose.sim.yml. See README.md "Simulation".
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ---- Gazebo Harmonic + ROS2 packages ---------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release ca-certificates \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-humble-ros-gzharmonic \
        ros-humble-rtabmap-ros \
        ros-humble-stereo-image-proc \
        ros-humble-image-transport \
        ros-humble-image-transport-plugins \
        ros-humble-camera-calibration \
        ros-humble-rviz2 \
        ros-humble-tf2-ros \
        ros-humble-tf2-tools \
        ros-humble-robot-state-publisher \
        ros-humble-teleop-twist-keyboard \
        ros-humble-rqt-image-view \
        python3-pip python3-pytest python3-colcon-common-extensions python3-lxml \
        python3-numpy python3-opencv python3-yaml \
        build-essential cmake git rapidjson-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libopencv-dev \
        # browser desktop
        tigervnc-standalone-server tigervnc-tools novnc websockify openbox xterm \
        mesa-utils libgl1-mesa-dri x11-xserver-utils \
        # GStreamer test sources: exercise stereo_camera_node without cameras
        gstreamer1.0-plugins-base gstreamer1.0-tools \
    && rm -rf /var/lib/apt/lists/*

# ---- ArduPilot SITL (Rover, same tag as the FC: 4.7.2) ---------------------
RUN pip3 install --no-cache-dir empy==3.3.4 future pexpect ptyprocess dronecan \
    && git clone --depth 1 --branch Rover-4.7.2 --recurse-submodules --shallow-submodules \
        https://github.com/ArduPilot/ardupilot.git /opt/ardupilot \
    && cd /opt/ardupilot \
    && ./waf configure --board sitl \
    && ./waf rover \
    && cp build/sitl/bin/ardurover /usr/local/bin/ardurover \
    # keep Tools/ (sim_vehicle.py, default params), drop the large build tree
    && rm -rf build .git

# ---- ardupilot_gazebo plugin (Gazebo <-> SITL JSON bridge) ------------------
ENV GZ_VERSION=harmonic
RUN git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git /opt/ardupilot_gazebo \
    && cd /opt/ardupilot_gazebo && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j"$(nproc)"

# pymavlink pinned like the dev/Jetson images so everyone speaks the same API.
RUN pip3 install --no-cache-dir pymavlink==2.4.43 pyserial anthropic

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && usermod -aG dialout,video ${USERNAME} \
    && mkdir -p /workspace/ros2_ws/build /workspace/ros2_ws/install /workspace/ros2_ws/log \
    && chown -R ${USER_UID}:${USER_GID} /workspace

ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build \
    GZ_SIM_RESOURCE_PATH=/workspace/sim/gazebo/models:/workspace/sim/gazebo/worlds:/opt/ardupilot_gazebo/models \
    # Software OpenGL (llvmpipe): no GPU inside Docker Desktop.
    LIBGL_ALWAYS_SOFTWARE=1 \
    DISPLAY=:1

COPY --chmod=755 docker/sim-entrypoint.sh /usr/local/bin/sim-entrypoint.sh

USER ${USERNAME}
WORKDIR /workspace
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
    && echo '[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash' >> ~/.bashrc

ENTRYPOINT ["/usr/local/bin/sim-entrypoint.sh"]
CMD ["sleep", "infinity"]
