# Deployment container - built and run ON the Jetson itself (arm64).
# Normally built by scripts/jetson_setup.sh, which picks BASE_IMAGE for your L4T.
#
# Default base: dustynv/jetson-containers' ROS2 Humble, built from source on
# the original Nano's last BSP (JetPack 4.6.x = L4T R32.7.x, Ubuntu 18.04).
# Official ROS2 Humble needs Ubuntu 22.04, which the original Nano can't run.
#   https://github.com/dusty-nv/jetson-containers
#
# Quirks of the R32.7.1 base image (verified):
#   - ROS lives at /opt/ros/humble/install/setup.bash (NOT /opt/ros/humble/setup.bash)
#   - ROS runs on Python 3.6 -> pymavlink pinned to 2.4.43, the newest that builds
#   - includes cv_bridge, image_transport (+compressed), image_proc,
#     stereo_image_proc, camera_calibration; OpenCV (CUDA build, its CUDA libs
#     are mounted from the host by the nvidia runtime - so `import cv2` only
#     works on a real Jetson, not under qemu on a PC)
#   - does NOT include RTAB-Map (see README "SLAM on the Jetson")
#
# Build on the Jetson:   docker compose -f docker/docker-compose.jetson.yml build
ARG BASE_IMAGE=dustynv/ros:humble-ros-base-l4t-r32.7.1
FROM ${BASE_IMAGE}

ARG ROS_SETUP=/opt/ros/humble/install/setup.bash
ENV ROS_SETUP=${ROS_SETUP}

# The base image still lists packages.ros.org for bionic, whose signing key
# has expired -> every `apt-get update` fails. ROS is built from source in this
# image, so that repo isn't needed: drop it.
RUN grep -rl "packages.ros.org" /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null \
        | xargs -r sed -i '/packages.ros.org/d' \
    && apt-get update && apt-get install -y --no-install-recommends v4l-utils git \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir pymavlink==2.4.43 pyserial

# RPLidar A1 driver (Slamtec rplidar_ros, ROS 2 branch, pinned) in its own
# workspace so editing our code doesn't rebuild it.
RUN mkdir -p /opt/rplidar_ws/src && cd /opt/rplidar_ws/src \
    && git clone https://github.com/Slamtec/rplidar_ros.git -b ros2 \
    && cd rplidar_ros && git checkout 24cc9b6dea97e045bda1408eaa867ce730fd3fc3 \
    && cd /opt/rplidar_ws && /bin/bash -c "source ${ROS_SETUP} && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release" \
    && rm -rf build log

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && usermod -aG dialout,video ${USERNAME}

WORKDIR /workspace/ros2_ws
COPY ros2_ws/src ./src
COPY scripts /workspace/scripts

# truck_sim (Gazebo/SITL) and truck_planner (anthropic SDK, Python 3.10+) are PC-side.
RUN /bin/bash -c "source ${ROS_SETUP} && source /opt/rplidar_ws/install/setup.bash && colcon build --packages-skip truck_sim truck_planner" \
    && chown -R ${USERNAME}:${USERNAME} /workspace

USER ${USERNAME}
RUN echo "source ${ROS_SETUP}" >> ~/.bashrc \
    && echo "source /opt/rplidar_ws/install/setup.bash" >> ~/.bashrc \
    && echo "source /workspace/ros2_ws/install/setup.bash" >> ~/.bashrc

# The base image's ENTRYPOINT (/ros_entrypoint.sh) sources ROS; we add the workspace.
CMD ["/bin/bash", "-c", "source /opt/rplidar_ws/install/setup.bash && source /workspace/ros2_ws/install/setup.bash && exec ros2 launch truck_bringup truck.launch.py"]
