#!/bin/bash
# One-time setup of the truck's Jetson. Run ON the Jetson, from the repo:
#
#   git clone <this repo> ~/RCTruckAuto && cd ~/RCTruckAuto
#   ./scripts/jetson_setup.sh                 # everything (asks for sudo)
#   ./scripts/jetson_setup.sh --no-build      # skip the (long) docker image build
#   ./scripts/jetson_setup.sh --headless      # also boot to console (frees ~800 MB RAM)
#   ./scripts/jetson_setup.sh --check         # only report what's installed / missing
#
# Safe to re-run: every step checks first and only changes what's missing.
#
# What it does:
#   1. checks the board + L4T/JetPack version (see README "Jetson: which Linux")
#   2. apt packages (v4l-utils, git, curl, python3-pip, htop, nano, ...)
#   3. Docker: NVIDIA runtime as default, compose v2 plugin, your user in the docker group
#   4. groups: dialout (FC serial), video (cameras)
#   5. udev rule -> /dev/fc0 for the SEQURE H743 (scripts/install-serial-udev-rules.sh)
#   6. 4 GB swap file (the Nano's 4 GB RAM is not enough to build/run SLAM without it)
#   7. max performance power mode (nvpmodel MAXN) + jetson_clocks at boot
#   8. builds the truck image (docker/docker-compose.jetson.yml)
#   9. sanity checks: FC, cameras, CUDA OpenCV in the container
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_VERSION="v2.29.7"          # last compose v2 release tested with Docker 20.10 (JetPack 4.6)
SWAP_GB=4
DO_BUILD=1
HEADLESS=0
CHECK_ONLY=0
for arg in "$@"; do
    case "$arg" in
        --no-build) DO_BUILD=0 ;;
        --headless) HEADLESS=1 ;;
        --check) CHECK_ONLY=1 ;;
        -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
        *) echo "unknown option $arg"; exit 2 ;;
    esac
done

c_ok()   { echo -e "  \e[32mOK\e[0m    $*"; }
c_warn() { echo -e "  \e[33mWARN\e[0m  $*"; }
c_fail() { echo -e "  \e[31mFAIL\e[0m  $*"; }
step()   { echo -e "\n\e[1m== $*\e[0m"; }
SUDO=sudo; [ "$(id -u)" = 0 ] && SUDO=""
TARGET_USER="${SUDO_USER:-$USER}"

# ---------------------------------------------------------------- 1. board
step "1. Board and OS"
if [ "$(uname -m)" != "aarch64" ] || [ ! -f /etc/nv_tegra_release ]; then
    c_fail "This is not a Jetson (uname -m = $(uname -m), no /etc/nv_tegra_release)."
    echo "        Run this on the truck's Jetson. On the PC use docker/docker-compose.sim.yml."
    exit 1
fi
MODEL="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo unknown)"
L4T_MAJOR="$(sed -n 's/^# R\([0-9]*\) (release).*/\1/p' /etc/nv_tegra_release)"
L4T_MINOR="$(sed -n 's/.*REVISION: \([0-9.]*\).*/\1/p' /etc/nv_tegra_release)"
L4T="R${L4T_MAJOR}.${L4T_MINOR}"
. /etc/os-release
echo "  board:  $MODEL"
echo "  L4T:    $L4T   ($PRETTY_NAME)"
echo "  RAM:    $(free -m | awk '/Mem:/{print $2}') MB"

case "$L4T_MAJOR" in
    32)
        BASE_IMAGE="dustynv/ros:humble-ros-base-l4t-r32.7.1"
        if [[ "$L4T_MINOR" != 7.* ]]; then
            c_warn "L4T $L4T: the image is built for R32.7.x (JetPack 4.6.1+). Upgrade JetPack"
            echo "        (sudo apt update && sudo apt dist-upgrade) or flash JetPack 4.6.x."
        else
            c_ok "JetPack 4.6.x / L4T R32.7 - the supported setup for the original Nano"
        fi ;;
    35)
        BASE_IMAGE="dustynv/ros:humble-ros-base-l4t-r35.4.1"
        c_warn "JetPack 5 (L4T $L4T): untested here. Using $BASE_IMAGE - check that tag exists"
        echo "        on hub.docker.com/r/dustynv/ros/tags for your exact L4T." ;;
    36)
        BASE_IMAGE="dustynv/ros:humble-ros-base-l4t-r36.2.0"
        c_warn "JetPack 6 (L4T $L4T, Orin): untested here. Using $BASE_IMAGE - check the tag."
        echo "        (Ubuntu 22.04 could also run ROS2 Humble natively from apt.)" ;;
    *)
        c_fail "Unknown L4T $L4T - see README 'Jetson: which Linux'."; exit 1 ;;
esac

if [ "$CHECK_ONLY" = 1 ]; then
    step "Status"
    command -v docker >/dev/null && c_ok "docker $(docker --version | cut -d' ' -f3 | tr -d ,)" || c_fail "docker missing"
    docker compose version >/dev/null 2>&1 && c_ok "docker compose $(docker compose version --short)" || c_fail "docker compose plugin missing"
    grep -q '"default-runtime": *"nvidia"' /etc/docker/daemon.json 2>/dev/null && c_ok "nvidia default runtime" || c_warn "nvidia not the default docker runtime"
    for g in docker dialout video; do id -nG "$TARGET_USER" | grep -qw $g && c_ok "$TARGET_USER in $g" || c_warn "$TARGET_USER not in group $g"; done
    [ -e /etc/udev/rules.d/99-fc-serial.rules ] && c_ok "FC udev rule" || c_warn "FC udev rule missing"
    [ -e /dev/fc0 ] && c_ok "/dev/fc0 present (FC plugged in)" || c_warn "/dev/fc0 not present (FC unplugged?)"
    [ -e /dev/rplidar ] && c_ok "/dev/rplidar present" || c_warn "/dev/rplidar not present (lidar unplugged? optional)"
    echo "  swap: $(free -m | awk '/Swap:/{print $2}') MB"
    ls /dev/video* 2>/dev/null | sed 's/^/  camera: /' || c_warn "no /dev/video* cameras"
    docker image inspect rctruck/truck:humble >/dev/null 2>&1 && c_ok "image rctruck/truck:humble built" || c_warn "image not built"
    exit 0
fi

# ---------------------------------------------------------------- 2. apt
step "2. System packages"
$SUDO apt-get update
$SUDO apt-get install -y --no-install-recommends \
    git curl ca-certificates v4l-utils python3-pip htop nano usbutils \
    gstreamer1.0-tools x11-xserver-utils
c_ok "apt packages"

# ---------------------------------------------------------------- 3. docker
step "3. Docker + NVIDIA runtime + compose"
if ! command -v docker >/dev/null; then
    c_warn "docker missing (JetPack normally ships it) - installing nvidia-docker2"
    $SUDO apt-get install -y nvidia-docker2
fi
if ! grep -q '"default-runtime": *"nvidia"' /etc/docker/daemon.json 2>/dev/null; then
    [ -f /etc/docker/daemon.json ] && $SUDO cp /etc/docker/daemon.json /etc/docker/daemon.json.bak.$(date +%s)
    $SUDO tee /etc/docker/daemon.json >/dev/null <<'EOF'
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
}
EOF
    $SUDO systemctl restart docker
    c_ok "nvidia set as default docker runtime (old config backed up)"
else
    c_ok "nvidia already the default runtime"
fi
if ! docker compose version >/dev/null 2>&1 && ! $SUDO docker compose version >/dev/null 2>&1; then
    $SUDO mkdir -p /usr/local/lib/docker/cli-plugins
    $SUDO curl -fsSL -o /usr/local/lib/docker/cli-plugins/docker-compose \
        "https://github.com/docker/compose/releases/download/${COMPOSE_VERSION}/docker-compose-linux-aarch64"
    $SUDO chmod +x /usr/local/lib/docker/cli-plugins/docker-compose
    c_ok "docker compose ${COMPOSE_VERSION} installed"
else
    c_ok "docker compose present"
fi

# ---------------------------------------------------------------- 4. groups
step "4. User groups"
for g in docker dialout video; do
    if id -nG "$TARGET_USER" | grep -qw "$g"; then c_ok "$TARGET_USER already in $g"
    else $SUDO usermod -aG "$g" "$TARGET_USER"; c_ok "added $TARGET_USER to $g (log out/in to apply)"; NEED_RELOGIN=1; fi
done

# ---------------------------------------------------------------- 5. udev
step "5. FC serial port (/dev/fc0)"
"$REPO/scripts/install-serial-udev-rules.sh"
c_ok "udev rule installed"

# ---------------------------------------------------------------- 6. swap
step "6. Swap (${SWAP_GB} GB file)"
SWAP_MB="$(free -m | awk '/Swap:/{print $2}')"
if [ -f /swapfile ] || [ "$SWAP_MB" -ge $((SWAP_GB * 1024 + 1500)) ]; then
    c_ok "swap already configured (${SWAP_MB} MB)"
else
    $SUDO fallocate -l ${SWAP_GB}G /swapfile
    $SUDO chmod 600 /swapfile
    $SUDO mkswap /swapfile >/dev/null
    $SUDO swapon /swapfile
    grep -q '^/swapfile' /etc/fstab || echo '/swapfile none swap sw 0 0' | $SUDO tee -a /etc/fstab >/dev/null
    c_ok "/swapfile ${SWAP_GB} GB enabled (in addition to the default zram)"
fi

# ---------------------------------------------------------------- 7. power
step "7. Power mode"
if command -v nvpmodel >/dev/null; then
    $SUDO nvpmodel -m 0 && c_ok "nvpmodel MAXN (mode 0) - needs a 5V/4A barrel supply or a good BEC on the truck"
    if [ ! -f /etc/systemd/system/jetson-clocks.service ]; then
        $SUDO tee /etc/systemd/system/jetson-clocks.service >/dev/null <<'EOF'
[Unit]
Description=Max out Jetson clocks for the truck stack
After=nvpmodel.service

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks

[Install]
WantedBy=multi-user.target
EOF
        $SUDO systemctl daemon-reload
        $SUDO systemctl enable jetson-clocks.service >/dev/null
        c_ok "jetson_clocks enabled at boot"
    else
        c_ok "jetson_clocks service already present"
    fi
else
    c_warn "nvpmodel not found - skipping"
fi
if [ "$HEADLESS" = 1 ]; then
    $SUDO systemctl set-default multi-user.target
    c_ok "boots to console from next reboot (undo: sudo systemctl set-default graphical.target)"
fi

# ---------------------------------------------------------------- 8. image
step "8. Truck docker image"
export RCTRUCK_BASE_IMAGE="$BASE_IMAGE"
DOCKER="docker"; docker info >/dev/null 2>&1 || DOCKER="$SUDO docker"
if [ "$DO_BUILD" = 1 ]; then
    echo "  building with base $BASE_IMAGE (first time: pulls ~3 GB, 10-20 min)"
    (cd "$REPO" && RCTRUCK_BASE_IMAGE="$BASE_IMAGE" $DOCKER compose -f docker/docker-compose.jetson.yml build)
    c_ok "image rctruck/truck:humble built"
else
    c_warn "skipped (--no-build). Later: docker compose -f docker/docker-compose.jetson.yml build"
fi

# ---------------------------------------------------------------- 9. checks
step "9. Sanity checks"
[ -e /dev/rplidar ] && c_ok "RPLidar found at /dev/rplidar -> $(readlink -f /dev/rplidar)" \
                    || c_warn "RPLidar not found (/dev/rplidar appears when it's plugged in; optional)"
[ -e /dev/fc0 ] && c_ok "FC found at /dev/fc0 -> $(readlink -f /dev/fc0)" \
                || c_warn "FC not found (plug in the SEQURE H743 USB; /dev/fc0 appears automatically)"
if ls /dev/video* >/dev/null 2>&1; then
    v4l2-ctl --list-devices 2>/dev/null | sed 's/^/        /'
else
    c_warn "no V4L2 cameras (/dev/video*) yet"
fi
if [ "$DO_BUILD" = 1 ]; then
    if $DOCKER run --rm --entrypoint bash rctruck/truck:humble -c \
        'source $ROS_SETUP && python3 -c "import cv2, pymavlink; bi=cv2.getBuildInformation(); print(\"cv2\", cv2.__version__, \"gstreamer:\", \"GStreamer:                   YES\" in bi)"' ; then
        c_ok "OpenCV + pymavlink load inside the container"
    else
        c_fail "OpenCV/pymavlink failed to load in the container"
    fi
    if [ -e /dev/fc0 ]; then
        $DOCKER run --rm --entrypoint bash -v /dev:/dev --device-cgroup-rule='c 166:* rmw' \
            rctruck/truck:humble -c 'python3 /workspace/scripts/fc_check.py --conn /dev/fc0 --seconds 3' \
            && c_ok "FC link healthy" || c_warn "fc_check reported problems (see above)"
    fi
fi

step "Done"
echo "  Start the truck stack:   docker compose -f docker/docker-compose.jetson.yml up -d"
echo "  Calibrate the cameras:   see README 'Stereo cameras' (scripts/stereo_calibrate.py)"
[ "${NEED_RELOGIN:-0}" = 1 ] && echo -e "  \e[33mLog out and back in\e[0m (or reboot) so the new groups apply."
exit 0
