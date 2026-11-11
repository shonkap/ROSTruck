# RCTruckAuto

Autonomy stack for a **Redcat Ground Pounder** (front + rear steer) RC truck:

- **Flight controller:** SEQURE H743 running **ArduRover 4.7.2**, speaking
  MAVLink over USB (`1209:5741 "Generic SequreH743"`, `/dev/ttyACM0`).
  It owns the servos/ESC. Rear steer is configured on the FC as a second
  reversed steering output (`SERVO1_FUNCTION=26`, `SERVO2_FUNCTION=26` +
  `SERVO2_REVERSED=1`, throttle on `SERVO3_FUNCTION=70`), so the FC mixes
  front/rear steering itself.
- **Compute:** **Jetson Nano 4 GB** running **ROS2 Humble** (in Docker), two
  cameras for stereo vision. **No wheel encoders** - odometry is visual
  (stereo VO) + the FC's IMU.
- **Simulation:** ArduPilot SITL (same 4.7.2 firmware) + Gazebo Harmonic with
  a model of the truck and a textured room to map.

## Quick start

**Simulation** (on the workstation, nothing plugged in):

```bash
# 1. start the sim container (first time builds the image, ~10 min)
docker compose -f docker/docker-compose.sim.yml up --build -d

# 2. open a shell in it and build the ROS workspace
docker compose -f docker/docker-compose.sim.yml exec sim bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash

# 3. start everything: Gazebo + ArduPilot SITL + fc_bridge + SLAM + tasks + web
ros2 launch truck_sim sim.launch.py

# 4. in a SECOND shell (docker compose -f docker/docker-compose.sim.yml exec sim bash):
#    be the RC pilot - flip the FC to GUIDED and arm it
ros2 run truck_sim sim_ctl guided arm
```

Then open:

| URL | What |
|---|---|
| **http://localhost:8080** | **Truck web page** - live map, click to add waypoints, *Drive waypoints*, *Map room*, STOP, task queue, planner requests |
| http://localhost:6080/vnc.html | Gazebo 3D view + RViz (browser desktop) |

Click **Map room** on the web page and watch it explore. Stop everything
with `./sim/stop_sim.sh` (inside the container), or `docker compose -f
docker/docker-compose.sim.yml down` (from the host) to shut the container.

**Real truck** (FC on USB, from the host): `.venv/bin/python scripts/fc_check.py`.
**Jetson**: `./scripts/jetson_setup.sh`, then
`docker compose -f docker/docker-compose.jetson.yml up -d` - web page at
`http://<jetson-ip>:8080`. Details below.

## Repo layout

```
ros2_ws/src/fc_bridge/      MAVLink <-> ROS2 bridge (telemetry, cmd_vel -> FC)
ros2_ws/src/truck_bringup/  truck + sim shared: stereo camera driver, SLAM launch,
                            calibration check, map saver, truck.launch.py
ros2_ws/src/truck_sim/      sim only: launch, stereo relay, sim_ctl, explore_room, turn_test
ros2_ws/src/truck_tasks/    task bus: task manager, navigator ("goto"), explorer ("map_room")
ros2_ws/src/truck_web/      web page plugin (map, waypoints, tasks, planner requests)
sim/gazebo/                 truck model + room generators (make_*.py), worlds
sim/params/                 SITL params + snapshot of the real FC's params
sim/fake_fc.py              tiny fake FC (no physics) for quick bridge tests
scripts/fc_check.py         MAVLink link health check (real FC or SITL)
scripts/stereo_calibrate.py guided stereo camera calibration
scripts/jetson_setup.sh     one-shot Jetson installer
docker/                     dev, sim (x86) and Jetson (arm64) images + compose files
```

## 1. Check the link to the FC

```bash
.venv/bin/python scripts/fc_check.py                     # truck plugged into this PC
.venv/bin/python scripts/fc_check.py --save-params fc.parm
```

Read-only (never moves anything). Checks heartbeat, firmware, telemetry
rates, corrupt packets, a parameter round trip, and the truck-specific params.
Result on 2026-11-07: **link healthy** - 4.7.2, all streams at the requested
rate, 0 bad packets, 60 ms param round trip, steering/throttle mapping correct.
It flags two **FC settings to fix before autonomy** (see section 6).

## 2. Simulation (workstation)

Everything runs in one container; the GUIs show up in your browser.

```bash
docker compose -f docker/docker-compose.sim.yml up --build -d   # first build ~10 min
docker compose -f docker/docker-compose.sim.yml exec sim bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
# tests:  python3 -m pytest -p no:anyio src/*/test
ros2 launch truck_sim sim.launch.py             # Gazebo + SITL + fc_bridge + SLAM + RViz
```

Open **http://localhost:6080/vnc.html** for the Gazebo and RViz windows.
Then, in a second shell (`docker compose ... exec sim bash`):

```bash
ros2 run truck_sim sim_ctl guided arm           # what the pilot's mode switch does on the truck
ros2 run truck_sim explore_room                 # automatic mapping drive (~1 min)
#   or drive yourself:  ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run truck_bringup map_saver --ros-args -p output:=/workspace/maps/room
```

`./sim/stop_sim.sh` stops everything. Launch options:

| option | default | |
|---|---|---|
| `indoor:=true` | false | **no GPS**: the FC navigates on the SLAM pose (like the real truck indoors) |
| `gui:=false rviz:=false` | true | headless, faster |
| `slam:=false` | true | no RTAB-Map |
| `tasks:=false` | true | no task manager / navigator / explorer / planner / web page |
| `guard:=false` | true | no stereo collision guard |
| `detector:=false` | true | no object detector (demo: finds the red bin) |
| `detector_rules:='[...]'` | `[]` | detection → task rules, e.g. `[{"label": "red_bin", "max_distance": 1.2, "task": {"type": "stop", "priority": 10}}]` |
| `map_name:=... localization:=true` | sim_room | saved map folder / relocalize in it (see *Saved maps*) |
| `auto_apply:=true` | false | steering self-check writes its fixes to the (simulated) FC |
| `world:=.../empty.sdf` | room | open floor for tuning (`ros2 run truck_sim turn_test`) |

The web page also shows the live left camera and the distance to the nearest
obstacle ahead.
A ground station on the host (QGroundControl / Mission Planner / MAVProxy)
can connect to **TCP 127.0.0.1:5762**.

**What's simulated, and how it maps to the truck:**

```
cmd_vel -> fc_bridge -> MAVLink TCP 5760 -> ArduRover SITL 4.7.2 (EKF3)
        -> PWM SERVO1 (front steer) / SERVO2 (rear steer, reversed) / SERVO3 (throttle)
        -> Gazebo: 4 steering knuckles + 4 driven wheels -> physics
Gazebo IMU -> SITL;  2 cameras (10 cm baseline, 640x480 @ 15 Hz) -> stereo_relay
        -> stereo/{left,right}/image_raw -> RTAB-Map stereo odometry + SLAM -> map
```

Same topics, TF frames, FC firmware, servo mapping and `WP_SPEED` as the real
truck, so code tested here runs unchanged on the Jetson. Things that differ:
the sim has GPS (GUIDED works indoors, see 6), perfect camera calibration,
and the steering tuning in `sim/params/ground_pounder_sitl.parm` is for the
model, not the real truck. Truck/camera dimensions are approximations - edit
`sim/gazebo/models/ground_pounder/make_model.py` and re-run it.

Docker Desktop has no GPU in containers, so Gazebo renders on the CPU
(llvmpipe). It still runs at real-time factor ~1.0 headless on this machine.

## 3. Jetson: which Linux

| Board | Required OS | ROS2 Humble via | Notes |
|---|---|---|---|
| **Jetson Nano 4 GB (original, B01/A02)** | **JetPack 4.6.x = L4T R32.7.x, Ubuntu 18.04** (newest JetPack the Nano supports: 4.6.4 / R32.7.4 or 4.6.1+) | Docker image `dustynv/ros:humble-ros-base-l4t-r32.7.1` | the supported setup; ROS runs on **Python 3.6** there |
| Jetson Orin Nano / NX (if you upgrade) | JetPack 6.x = L4T R36, Ubuntu 22.04 | Docker (`dustynv/ros:humble-...-l4t-r36.x`) or native apt | the setup script picks the base image, but this path is untested |

Flash the Nano with the **JetPack 4.6.x SD card image** from NVIDIA (use a
64 GB+ A2/U3 card, or boot from USB SSD). Don't run `do-release-upgrade` -
Ubuntu 20.04/22.04 are not supported on the original Nano; ROS2 Humble runs
in Docker instead. Power it from the barrel jack (5 V 4 A, jumper J48) or a
good 5 V BEC on the truck - micro-USB power browns out with cameras + MAXN.

## 4. Jetson setup

On the Jetson, after flashing:

```bash
git clone <this repo> ~/RCTruckAuto && cd ~/RCTruckAuto
./scripts/jetson_setup.sh          # asks for sudo; safe to re-run
./scripts/jetson_setup.sh --check  # just report status
```

It installs/configures: apt basics (`v4l-utils`, ...), Docker with the NVIDIA
runtime as default, the compose plugin, groups (`docker`, `dialout`,
`video`), the `/dev/fc0` udev rule, a 4 GB swap file, MAXN power mode +
`jetson_clocks` at boot, builds the `rctruck/truck:humble` image, and checks
the FC (runs `fc_check.py` in the container) and the cameras.
`--headless` also switches the Nano to boot without the desktop (~800 MB RAM back;
keep the desktop until the cameras are calibrated).

Then:

```bash
docker compose -f docker/docker-compose.jetson.yml up -d     # fc_bridge + cameras + TF
docker compose -f docker/docker-compose.jetson.yml logs -f
```

## 5. Stereo cameras

**Hardware advice.** Two identical cameras, rigidly mounted, parallel, same
height, 6-12 cm apart. A global-shutter or a single "stereo module" with
hardware sync is best for a moving truck; two separate USB webcams work but
are not synchronised: the driver buffers a few frames per camera and pairs
the closest-in-time ones (within half a frame period, ~33 ms at 15 fps -
two free-running cameras can't do better), and reports the skew. Lock focus and auto-exposure if the cameras allow it.
Supported sources (`ros2_ws/src/truck_bringup/config/cameras.yaml`):
two USB cams (`"0"`, `"2"`), two CSI cams (`csi:0`, `csi:1`), one side-by-side
USB stereo camera (`mode: side_by_side`), or any GStreamer pipeline.

**Calibrate** (the wizard walks you through each step):

```bash
python3 scripts/stereo_calibrate.py board      # printable checkerboard PNG (9x6, 25 mm)
python3 scripts/stereo_calibrate.py cameras    # which /dev/video* is which
python3 scripts/stereo_calibrate.py run        # guided calibration
python3 scripts/stereo_calibrate.py check      # re-check later
```

`run` starts the cameras, checks both stream, **checks left/right aren't
swapped** (cover the left lens), opens ROS2's `cameracalibrator` with
instructions, saves `left.yaml`/`right.yaml` to
`ros2_ws/src/truck_bringup/config/camera_info/` (the calibrator's COMMIT
button writes there directly), then runs `stereo_check`: rectification
(epipolar) error, baseline vs. your measurement, depth resolution, disparity
coverage, and a preview image. On the Jetson, run it inside the container
(it needs a monitor on the Jetson, `xhost +local:` first):

```bash
docker compose -f docker/docker-compose.jetson.yml run --rm truck \
    python3 /workspace/scripts/stereo_calibrate.py run
```

No monitor on the Jetson? Start the cameras there with
`ros2 launch truck_bringup cameras.launch.py publish_compressed:=true` and run
`stereo_calibrate.py run --remote` on the PC (decompresses the JPEG stream
locally; WiFi can't carry two raw streams).

`stereo_check` was verified against the sim's cameras.

## 6. Before letting the Jetson drive the real truck

FC settings - **applied on 2026-11-07** with `scripts/fc_set_params.py --profile safety`
(full backup + log in `backups/`; undo with `--restore backups/fc_params_<date>.parm`):

- `ARMING_REQUIRE 1` (was 0 = always armed): arm deliberately - throttle at
  zero and steering stick held right (`ARMING_RUDDER 2`), or from a GCS.
- Mode switch on channel 8: low = MANUAL (`MODE1/2`), middle = HOLD
  (`MODE3/4`), high = GUIDED (`MODE5/6`) - the Jetson only drives in GUIDED.
- Still worth considering: `FS_GCS_ENABLE` (no failsafe if the Jetson dies;
  GUIDED just times out and stops after ~3 s).
- **GUIDED indoors:** ArduRover's GUIDED mode needs a position estimate
  (`requires_position()` in Rover 4.7.2), and indoors there's no GPS. So
  `fc_bridge` can feed the SLAM pose to the FC as external navigation - see
  *Indoor driving (no GPS)* below. Until that's set up on the real FC, map
  rooms by driving with the transmitter in MANUAL while SLAM runs.

## Indoor driving (no GPS)

`fc_bridge` with `vision_pose: true` sends the SLAM pose (`map -> base_link`)
to the FC at 15 Hz as `VISION_POSITION_ESTIMATE` + `VISION_SPEED_ESTIMATE`
(ENU -> NED converted, SLAM jumps signalled with `reset_counter`, stale poses
never sent), plus the EKF origin and system time the FC needs without GPS.
The FC's EKF then navigates on SLAM and accepts GUIDED.

FC parameters for the real truck (same as `sim/params/indoor_extnav.parm`);
`scripts/fc_set_params.py --profile indoor` sets them (with backup), or use Mission Planner/QGC:

```
GPS1_TYPE 0        VISO_TYPE 1        VISO_DELAY_MS 50
EK3_SRC1_POSXY 6   EK3_SRC1_VELXY 6   EK3_SRC1_POSZ 1   EK3_SRC1_VELZ 0   EK3_SRC1_YAW 6
COMPASS_USE 0      (heading from SLAM; the compass is unreliable indoors)
```

then reboot the FC and launch with `vision_pose: true` in
`fc_bridge/config/fc_bridge.yaml` (SLAM must be running).

Verified in the sim with GPS removed (`ros2 launch truck_sim sim.launch.py
indoor:=true`): EKF3 reports "using external nav data", GUIDED + arm are
accepted, *Map room* completes, and the FC's own EKF position matched the
SLAM pose to the centimetre.

## Lidar (RPLidar A1)

Mounted 260 mm off the ground near the front, scanning 360°. Keep the
cameras **below** its scan plane - anything in the plane blocks the beams
(in the sim the stereo bar sits at ~0.24 m for that reason).

What it adds (all on by default in the sim, `lidar:=true` on the truck):

- **Lidar odometry** (`odom:=auto` picks it when a lidar is present): RTAB-Map
  `icp_odometry` scan matching. Works on blank walls and in the dark. On the
  same 19 m sim drive it was ~3x more accurate than stereo visual odometry -
  ATE rms 0.6 cm vs 1.8 cm, max 1.6 vs 4.1 cm (`scripts/sim_odom_compare.sh`).
  The sim lidar is idealised (perfectly flat walls, no smear while the A1's
  5.5 Hz head rotates during turns), so expect a smaller gap on the real
  truck - compare with `odom:=stereo` once it's mounted.
- **One fused map**: RTAB-Map subscribes to the scan as well as the stereo
  pair - loop closures and odometry links are refined with scan matching, the
  2D grid takes walls from the lidar and low obstacles from stereo.
- **360° collision guard** (`scan_guard`): caps forward AND reverse speed by
  the nearest obstacle in the truck's corridor ahead of / behind the bumpers;
  `fc_bridge` applies the most restrictive fresh cap from both guards. The
  navigator's three-point turns use the measured rear distance. The web page
  shows *ahead* (camera / lidar) and *behind*.
- Outdoors it mostly stays an obstacle sensor: few walls in range, the single
  scan plane hits slopes, and the A1 struggles in direct sun.

On the truck: `./scripts/install-serial-udev-rules.sh` (adds `/dev/rplidar`),
then `ros2 launch truck_bringup truck.launch.py lidar:=true` (the Jetson image
builds Slamtec's `rplidar_ros`). Check the scan direction in RViz - an object
in front of the truck must show up at +x; if it's backwards set `lidar_yaw:
3.14159` in `truck_bringup/config/rig.yaml`.

## Safety: collision guard

`collision_guard` (truck_bringup) computes stereo depth in the corridor in
front of the truck and publishes `safety/speed_limit`; `fc_bridge` caps the
forward speed of **every** `cmd_vel` with it (0 inside 0.4 m, ramp to full at
1.2 m; reversing stays allowed so recoveries work; a stale limit is ignored
with a warning). Also `safety/obstacle_distance`, shown on the web page. On
the truck it needs rectified images (`slam:=true`). The RC transmitter
remains the real kill switch.
- First cmd_vel test: wheels off the ground. Positive `angular.z` must turn
  left with the rear wheels counter-steering.

## Tasks, web page and plugins

Everything the truck *does* is a **task** on one queue. Tasks come from
anywhere (web page, an LLM planner, an object detector, the CLI) and are
executed by whichever **module** registered for that task type. All of it is
JSON on plain `std_msgs/String` topics, so a plugin is just a node (any
language, any machine on the network) that publishes/subscribes to these:

| Topic | Dir | Content |
|---|---|---|
| `tasks/submit` | → manager | `{"type": "goto", "waypoints": [[x, y], ...]}`, `{"type": "map_room"}`, `{"type": "stop"}`, `{"type": "request", "text": "..."}` + optional `priority` (0-10, higher preempts), `source` |
| `tasks/cancel` | → manager | `"all"` or a task id |
| `tasks/status` | manager → | queue, active task, history, registered modules (latched) |
| `tasks/modules` | module → | `{"name": "navigator", "handles": ["goto"]}` every 2 s - this is how a plugin adds a task type |
| `tasks/dispatch` / `tasks/abort` | manager → module | start / stop a task (`handler` = module name) |
| `tasks/feedback` | module → | `{"id": 3, "state": "running" / "succeeded" / "failed", "message", "progress"}` |
| `tasks/requests` | manager → planners | natural-language requests (`type: request`) for an LLM planner |
| `tasks/reports` | planner → | `{"request_id", "title", "text", "tasks": [...]}` - shown on the web page |

Built-in modules (`ros2 launch truck_tasks tasks.launch.py`, included in
the sim and truck launches):

- **task_manager** - queue, priorities/preemption, `stop`, forwards `request`s,
  fails tasks nobody handles.
- **navigator** (`goto`) - A* on the SLAM `/map` with the walls inflated by
  the truck's radius, car-like pure pursuit (no turning on the spot; turn
  rate capped by the minimum turn radius, per side, updated from the
  steering self-check). Also serves `nav/goal` for other modules.
- **explorer** (`map_room`) - warm-up lap, then frontier exploration
  (drive to the edge of the known map) until nothing is left or
  `max_minutes`; saves the map. Only frontiers reachable through known free
  space count (stereo noise makes fake free patches beyond walls), and goals
  keep a 0.45 m standoff from obstacles (stereo can't see closer than ~0.25 m).
- Speed adapts to the path: up to `max_speed` (0.5 m/s) on straight
  stretches, `speed` (0.3 m/s) in curves, slowing for the last 0.6 m.
- It's a car, it can't pivot: when the path leads behind the truck (> 100°)
  the navigator does a **three-point turn** - alternating reverse and forward
  legs on full lock (both rotate the truck the same way), switching early when
  that direction is blocked (an obstacle in the map just behind, or the
  collision guard seeing something close ahead), and waiting if boxed in.
- Recovery: no progress for 12 s → reverse 0.5 m and replan; visual odometry
  lost (cameras too close to something) → back up slowly to regain tracking,
  fail the task after 30 s with a clear message.
- Since the forward cameras never see the floor around the truck, the
  explorer counts the floor it has driven over as free.

Verified in the sim via the web API: *Map room* visited 8 frontiers and
mapped 30.4 m² of the 30 m² room, then finished on its own ("no frontiers
left"); a queued *goto* then drove home, including a three-point turn for a
target 174° behind the truck (arrived within 0.12 m).
- **web_node** (`truck_web`) - the page on port 8080. No login: keep it on
  your own network, or launch with `web_token:=<secret>` and open
  `http://host:8080/?token=<secret>`.

Tasks only move the truck while the FC is **GUIDED and armed** (the pilot's
switch); otherwise they wait. The transmitter always wins.

**LLM planner** (`truck_planner`, PC-side - needs Python 3.10 + the
`anthropic` SDK, so not on the Jetson; it reaches the truck over ROS like any
plugin). Type a request into *Ask the planner* on the web page. The planner
sends Claude (`claude-opus-5-5`, structured JSON output, server-side refusal
fallback) the truck's situation - pose, map summary and unexplored frontiers,
landmarks, FC state, task queue - and gets back a report + tasks. Every task
is validated (known type, waypoints moved onto reachable free space next to
obstacles/landmarks). Tasks are only **proposed**: the web page shows *Run
these tasks*; `auto_submit:=true` queues them directly.

```bash
export ANTHROPIC_API_KEY=...       # in the shell/container running the planner
ros2 run truck_planner planner_node            # (sim.launch.py starts it already)
```

Exactly what the planner sends Claude is published on `planner/context`
(`ros2 topic echo /planner/context`) - inspect it before adding a key.
Without credentials it falls back to a small keyword planner ("map the room",
"go home", "go to 1.5, 2", "go to the <landmark>", "stop") and says so.

**Object detection** (`detector_node`, truck_bringup): detect objects in the
left image → range them with stereo → transform to the map → merge repeated
sightings into **landmarks** (`object_landmarks` topic, which the planner uses:
"go to the couch"; ranged by stereo, or by where the object meets the floor
when it has no texture). Rules turn detections into tasks, e.g.

```bash
ros2 run truck_bringup detector_node --ros-args -p backend:=onnx \
  -p model:=/models/yolov8n.onnx -p "classes:=[person, chair, couch]" \
  -p rules:='[{"label": "person", "max_distance": 1.0, "task": {"type": "stop", "priority": 10}}]'
```

The default `color` backend is a demo that finds the red bin in the sim room;
your trained model goes in as an ONNX file (OpenCV DNN, CUDA on the Jetson)
or as a new class with `detect(bgr) -> [Detection]` in `detection.py`.

Minimal plugin skeleton (Python, Jetson-compatible):

```python
from truck_tasks.ros_common import ModuleAnnouncer, jmsg   # registration + feedback helpers
class MyModule(Node):
    def __init__(self):
        super().__init__('my_module')
        self.task = ModuleAnnouncer(self, 'my_module', ['my_task'], self.start, self.abort)
    def start(self, task):      # dict from tasks/submit, plus "id"
        ...                      # do the work; report with
        self.task.feedback('succeeded', 'done')
    def abort(self):
        ...
```

From the command line:

```bash
ros2 topic pub --once /tasks/submit std_msgs/msg/String "{data: '{\"type\": \"map_room\"}'}"
ros2 topic echo /tasks/status
curl -X POST localhost:8080/api/task -d '{"type": "goto", "waypoints": [[1.0, 0.5]]}'
```

## More from the web page

- **Places** - *Save truck position* or *Save a spot on the map…* and give it a
  name ("kitchen door"). Places are drawn on the map, have a *Go* button, and
  work anywhere a waypoint does: `{"type": "goto", "waypoints": ["kitchen door"]}`
  (web, CLI, LLM planner, detector rules). Stored per map in `places.json`.
- **Drive (hold)** - an on-screen joystick (or the arrow keys). Hold to drive,
  let go to stop: commands expire after 0.3 s, so a dropped connection stops the
  truck. Max 0.4 m/s, refused while a task is running, collision guard still
  applies, and the FC must be GUIDED + armed.
- **Save map** - writes the current map as `map.png/.yaml` (nav2 format) into
  the map's folder; use it after mapping by driving with the transmitter.
- **Record** - starts/stops `ros2 bag record`: preset *slam* (stereo images +
  camera info + IMU + TF, ~25 MB/s - replay later to tune SLAM) or *light* (no
  images). Time limit and free-disk check; bags in `~/.ros/bags` on the truck,
  `bags/` in the sim.

## Saved maps and relocalization

Every map lives in its own folder (RTAB-Map database, `map.png/.yaml` from
*Map room*, `places.json`):

```bash
# map a room (or extend it with fresh_map:=false)
ros2 launch truck_bringup truck.launch.py slam:=true map_name:=living_room
# later: load it and just relocalize - no exploring needed
ros2 launch truck_bringup truck.launch.py slam:=true map_name:=living_room localization:=true
```

Where the truck starts matters for relocalization: by default (`start_at:=origin`)
put it back where you **started** mapping that room (a home/dock spot); use
`start_at:=last` if it's still where the previous session ended. RTAB-Map then
corrects the pose once it recognises the room.

Folders: `~/.ros/maps/<name>` on the truck (persistent docker volume),
`maps/<name>` in the repo for the sim (`ros2 launch truck_sim sim.launch.py
map_name:=... localization:=true`).

## Watchdog

`watchdog` (truck_tasks) turns bad conditions into tasks: battery below
`low_voltage` for 10 s while a task runs → return to the place called `home`
(or the map origin) at priority 9; below `critical_voltage` → stop everything;
FC link lost 3 s during a task → stop everything. Voltages default to 0 = off -
set them for your pack, e.g. 2S LiPo: `low_voltage:=7.0 critical_voltage:=6.6`
on `tasks.launch.py`.

## Changing FC parameters

`scripts/fc_set_params.py` downloads a **full backup** first
(`backups/fc_params_<time>.parm`), sets, reads back, and logs old → new to
`backups/fc_param_changes.log`:

```bash
.venv/bin/python scripts/fc_set_params.py --profile safety --dry-run   # see what would change
.venv/bin/python scripts/fc_set_params.py --profile indoor             # no-GPS ExtNav (then reboot FC)
.venv/bin/python scripts/fc_set_params.py WP_SPEED=1.5
.venv/bin/python scripts/fc_set_params.py --restore backups/fc_params_<time>.parm
```

## Testing

Inside the sim container:

```bash
./scripts/run_tests.sh               # build + unit tests + lint (~1 min)
python3 scripts/sim_smoke_test.py --indoor     # end to end: arm, Map room, go home (~3-5 min)
python3 scripts/sim_feature_test.py --indoor   # places, teleop, saved map + relocalization (~12 min)
```

## Performance notes

Measured in the sim (2026-11-08, CPU time per node during *Map room*, % of
one PC core): RTAB-Map stereo odometry ~35 %, RTAB-Map ~30 %, fc_bridge
~13 %, web page ~12 %, collision guard ~11 %, detector ~10 %, navigator ~8 %,
idle modules (task manager, watchdog, places, recorder) ~4 % each. A Jetson
Nano core is roughly 4-5x slower than this PC's, which is why SLAM runs on
the PC for now and the Python modules are kept light.

Sim-only: Gazebo publishes `/clock` every 1 ms physics step; `clock_relay`
re-publishes it at 100 Hz, which cut every sim-time Python node from ~30-45 %
to ~4-13 % of a core.

## Steering self-check (fc_bridge)

While the FC drives in GUIDED, `fc_bridge` compares the turn rate it asked
for with what the IMU measures and where the steering servos sit, and
publishes `fc/steering_report` (also on the web page). This truck is **not
skid steer** - one motor drives all four wheels the same way and it turns
only with the front + rear servos - so the fixable things are:

| Symptom | Fix |
|---|---|
| turns too little / too much in both directions | `ATC_STR_RAT_FF` scaled (max x1.5 per step) |
| curves when told to go straight, or servo sits off trim | `SERVO1_TRIM` / `SERVO2_TRIM` (max 25 µs per step) |
| steering at its end stop and still under-turning on one side | learns the minimum turn radius **per side**, clamps `cmd_vel` to it, sets `TURN_RADIUS`; warns if left and right differ (linkage / `SERVOn_MIN/MAX`) |

By default it only **recommends**. `auto_apply: true` (in
`fc_bridge/config/fc_bridge.yaml`, or `auto_apply:=true` on the sim launch)
writes the changes to the FC; every change is logged as old -> new in
`~/.ros/fc_param_changes.log` so it can be undone. Learned turn limits
persist in `~/.ros/steering_limits.yaml` (delete to re-learn). To give it
data quickly, drive straight lines and turns in an open space - or in the
sim: `ros2 run truck_sim turn_test --ros-args -p "rates:=[0.0, 0.35, 0.0, -0.35, 1.5, -1.5]" -p repeat:=4`.

Verified in the sim by breaking the truck on purpose (FF 0.4, front trim
+60 µs, right-side steering stop at half angle): it raised FF 0.4 → 0.6 →
0.88, walked the trim back toward centre, and learned left 0.31 m / right
0.61 m minimum radius (geometry says 0.29 / 0.63).

## fc_bridge reference

One node, `mavlink_bridge_node`, owns the MAVLink link (two processes on one
serial port corrupt each other). It reconnects if the USB cable drops.

| Topic | Type | Notes |
|---|---|---|
| `fc/imu` | `sensor_msgs/Imu` | 20 Hz, **FLU/ENU** (converted from ArduPilot FRD/NED), with covariances |
| `fc/gps/fix` | `sensor_msgs/NavSatFix` | 5 Hz from `GPS_RAW_INT` |
| `fc/battery` | `sensor_msgs/BatteryState` | 2 Hz |
| `fc/mode`, `fc/armed`, `fc/status` | `String`, `Bool`, `String` | from HEARTBEAT |
| `cmd_vel` (sub) | `geometry_msgs/Twist` | `linear.x` m/s, `angular.z` rad/s (+ = left) |

`cmd_vel` -> `SET_ATTITUDE_TARGET` (speed + turn rate, scaled by `WP_SPEED`),
**only while the FC is GUIDED and armed**; stop if `cmd_vel` is silent 0.5 s.
The node never arms or changes mode. Code in `ros2_ws` must stay
**Python 3.6-compatible** (Jetson image).

## SLAM

`truck_bringup/launch/slam.launch.py` - RTAB-Map stereo visual odometry
(3-DoF, ground vehicle) + `fc/imu` + mapping (2D occupancy grid + 3D cloud),
shared by sim and truck. The Jetson image doesn't include RTAB-Map yet (the
dustynv Humble/L4T R32 image has no build of it). For now run SLAM on the PC
from the truck's stream:

```bash
# Jetson
docker compose -f docker/docker-compose.jetson.yml run --rm truck \
    ros2 launch truck_bringup truck.launch.py publish_compressed:=true
# PC (sim or dev container on the same network, same ROS_DOMAIN_ID)
ros2 launch truck_bringup slam.launch.py compressed:=true
```

(The sim container uses Docker's bridge network; for live DDS with the
Jetson use `network_mode: host` - native Docker Engine - or run ROS2 on the
host.)

## Dev container

`docker/docker-compose.yml` (`dev` + `fake_fc`) is the VS Code Dev Container.
It used to be capped at 2 GB RAM to mimic the Nano, which got it **OOM-killed**
(exit 137) once VS Code Server + Pylance + the Claude Code extension loaded -
that was the "extension crashes on Reopen in Container". Now 8 GB.
**Real FC + containers on this workstation.** Docker Desktop can't pass USB
devices into containers. What works today, from the host:
`.venv/bin/python scripts/fc_check.py` (read-only health check) and
`scripts/fc_set_params.py` (changes with backup). Getting the FC *into* a
container (to run `fc_bridge` against the real truck from the PC):

- UDP via `scripts/fc_udp_forward.py` to the published port 14550 was tested on
  2026-11-07 and **no packets arrived** in the container on this machine.
- TCP should work: on the host `scripts/fc_udp_forward.py --udp tcpin:0.0.0.0:5770`,
  in the container `ros2 launch fc_bridge fc_bridge.launch.py
  connection:=tcp:host.docker.internal:5770`. It opens a listening port with
  the FC behind it - only do this on a trusted network (or firewall 5770).
- Or switch to native Docker Engine (then `devices: /dev/fc0` just works), or
  run on the Jetson, where the container gets `/dev/fc0` directly.

## Roadmap

1. ~~FC telemetry into ROS2~~ ✅  ~~cmd_vel -> FC~~ ✅ (bench-verify on the truck)
2. ~~SITL + Gazebo sim with stereo + SLAM~~ ✅  ~~steering self-check~~ ✅
3. ~~Task bus, navigator, map_room explorer, web page~~ ✅ (sim)
4. ~~SLAM pose -> FC (VISION_POSITION_ESTIMATE) so GUIDED works indoors~~ ✅ (sim; set the FC params on the truck)
5. ~~Collision guard, LLM planner plugin, object-detection plugin skeleton~~ ✅ (sim)
6. Calibrate the real stereo pair, map a real room (PC-side SLAM first)
7. RTAB-Map built for the Jetson image
8. Your detector model (ONNX) + detection rules; planner with an API key
