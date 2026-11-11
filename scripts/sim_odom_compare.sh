#!/bin/bash
# Compare odometry sources on the same drive (run inside the sim container):
#   ./scripts/sim_odom_compare.sh                 # stereo vs lidar
#   ./scripts/sim_odom_compare.sh lidar           # just one
# Each run: fresh sim (lidar fused into SLAM), GUIDED + arm, the explore_room
# figure-eight (~60 s, ~19 m), graded by odom_eval against Gazebo ground truth.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$REPO/ros2_ws/install/setup.bash"
MODES="${*:-stereo lidar}"
for mode in $MODES; do
    "$REPO/sim/stop_sim.sh" >/dev/null
    log=/tmp/odom_compare_$mode.log
    setsid ros2 launch truck_sim sim.launch.py gui:=false rviz:=false tasks:=false guard:=false \
        detector:=false lidar:=true odom:=$mode > "$log" 2>&1 &
    for _ in $(seq 90); do grep -aq "origin set" "$log" && break; sleep 2; done
    sleep 5
    for _ in 1 2 3 4; do
        timeout 60 ros2 run truck_sim sim_ctl guided arm 2>&1 | tail -1 | grep -q "armed=True" && break
        sleep 5
    done
    ros2 run truck_sim odom_eval --ros-args -p use_sim_time:=true -p seconds:=70.0 > /tmp/odom_eval_$mode.out 2>&1 &
    EVAL=$!
    sleep 2
    timeout 200 ros2 run truck_sim explore_room --ros-args -p use_sim_time:=true > /dev/null 2>&1
    wait $EVAL
    echo "odom:=$mode -> $(grep ODOM_EVAL /tmp/odom_eval_$mode.out)"
    grep -a -c "process has died" "$log" | sed 's/^/   crashed processes: /'
done
"$REPO/sim/stop_sim.sh" >/dev/null
