#!/bin/bash
# Stop everything the sim launch started (run inside the sim container).
pkill -INT -f "[r]os2 launch truck_sim" 2>/dev/null
for _ in $(seq 10); do pgrep -f "[r]os2 launch truck_sim|[a]rdurover|[g]z sim" >/dev/null || break; sleep 1; done
# Kill the launch parents FIRST - otherwise respawn=True nodes come straight back.
pkill -9 -f "[r]os2 launch truck_sim" 2>/dev/null
# Every ROS node started by launch carries --ros-args: catches nodes added
# later (task modules, plugins) without keeping a list of names here.
pkill -9 -f -- "[-]-ros-args" 2>/dev/null
pkill -9 -f "[a]rdurover|[g]z sim" 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
echo "sim stopped"
