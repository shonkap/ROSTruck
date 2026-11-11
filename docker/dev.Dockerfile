# Dev/sim container - runs on the x86 workstation, NOT on the Jetson.
#
# ROS2 Humble on Ubuntu 22.04. The Jetson image (docker/jetson.Dockerfile)
# runs the same ROS2 distro on the Nano's L4T R32.7.1 / Ubuntu 18.04 via
# dustynv/jetson-containers. One difference to keep in mind: ROS there runs on
# Python 3.6, so code in ros2_ws/ must stay Python 3.6 compatible
# (no dataclasses, walrus operator, `list[int]` hints, etc.).
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive
# Perception/SLAM packages, for developing the stereo + RTAB-Map pipeline
# against recorded bags on the workstation. Set to 0 for a slimmer image.
ARG WITH_PERCEPTION=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-pytest \
        python3-colcon-common-extensions \
        ros-humble-tf2-ros \
    && if [ "$WITH_PERCEPTION" = "1" ]; then apt-get install -y --no-install-recommends \
        ros-humble-stereo-image-proc \
        ros-humble-image-transport \
        ros-humble-rtabmap-ros \
        ros-humble-robot-localization ; fi \
    && rm -rf /var/lib/apt/lists/*

# Pinned to the newest pymavlink that still builds on the Jetson's Python 3.6,
# so both images speak exactly the same MAVLink dialect/API.
RUN pip3 install --no-cache-dir pymavlink==2.4.43 pyserial

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && usermod -aG dialout ${USERNAME} \
    # Pre-create the colcon dirs that docker-compose mounts as named volumes,
    # owned by the dev user - otherwise Docker creates them root-owned and
    # `colcon build` fails with "Permission denied".
    && mkdir -p /workspace/ros2_ws/build /workspace/ros2_ws/install /workspace/ros2_ws/log \
    && chown -R ${USER_UID}:${USER_GID} /workspace

USER ${USERNAME}
WORKDIR /workspace

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
    && echo '[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash' >> ~/.bashrc
CMD ["/bin/bash"]
