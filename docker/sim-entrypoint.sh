#!/bin/bash
# Starts a virtual X display (:1) exported to the browser via noVNC
# (http://localhost:6080/vnc.html), then runs the container command.
set -e
source /opt/ros/humble/setup.bash
[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash

if [ "${SIM_DESKTOP:-1}" = "1" ] && ! pgrep -x Xtigervnc >/dev/null; then
    rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
    Xtigervnc :1 -geometry "${SIM_RESOLUTION:-1600x900}" -depth 24 \
        -SecurityTypes None -localhost yes -AlwaysShared >/tmp/xvnc.log 2>&1 &
    for _ in $(seq 50); do [ -e /tmp/.X11-unix/X1 ] && break; sleep 0.1; done
    openbox >/tmp/openbox.log 2>&1 &
    xterm -geometry 120x30+0+0 -title "sim shell" -e bash >/dev/null 2>&1 &
    websockify --web /usr/share/novnc 6080 localhost:5901 >/tmp/novnc.log 2>&1 &
    echo "Desktop: http://localhost:6080/vnc.html"
fi
exec "$@"
