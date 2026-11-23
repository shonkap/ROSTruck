#!/usr/bin/env bash
# Run the truck software on the Jetson.
#
#   ./scripts/truck.sh start          start (or update to) what docker/truck.env says
#   ./scripts/truck.sh stop
#   ./scripts/truck.sh restart        after editing docker/truck.env
#   ./scripts/truck.sh logs           follow the output (Ctrl-C leaves it running)
#   ./scripts/truck.sh status
#   ./scripts/truck.sh shell          a shell in the running container (ros2 topic list, ...)
#   ./scripts/truck.sh autostart on   start by itself at power-up (battery plugged in)
#   ./scripts/truck.sh autostart off
#
# Starting is safe on the bench: fc_bridge never arms or changes mode - the
# truck only moves once the pilot switches the FC to GUIDED and arms.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE=(docker compose --env-file "$REPO/docker/truck.env" -f "$REPO/docker/docker-compose.jetson.yml")
SUDO=$([ "$(id -u)" = 0 ] && echo "" || echo sudo)

case "${1:-}" in
    start)   "${COMPOSE[@]}" up -d
             echo "started: $(grep -E '^TRUCK_LAUNCH_ARGS=' "$REPO/docker/truck.env")" ;;
    stop)    "${COMPOSE[@]}" down ;;
    restart) "${COMPOSE[@]}" up -d --force-recreate ;;
    logs)    "${COMPOSE[@]}" logs -f --tail 100 ;;
    status)  "${COMPOSE[@]}" ps
             if systemctl is-enabled rctruck.service >/dev/null 2>&1; then echo "autostart: on"; else echo "autostart: off"; fi ;;
    shell)   "${COMPOSE[@]}" exec truck bash ;;
    autostart)
        case "${2:-}" in
            on)
                # Docker comes up at boot by itself; this unit then starts the truck
                # once Docker is ready - no login needed, works headless.
                $SUDO tee /etc/systemd/system/rctruck.service >/dev/null <<UNIT_EOF
[Unit]
Description=RC truck software (docker compose, $REPO)
Requires=docker.service
After=docker.service network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=$REPO
ExecStart=$REPO/scripts/truck.sh start
ExecStop=$REPO/scripts/truck.sh stop
TimeoutStartSec=300

[Install]
WantedBy=multi-user.target
UNIT_EOF
                $SUDO systemctl daemon-reload
                $SUDO systemctl enable docker.service rctruck.service
                echo "autostart on: the truck software starts ~30-60 s after power-up."
                echo "Check after a reboot with: ./scripts/truck.sh status" ;;
            off)
                $SUDO systemctl disable rctruck.service 2>/dev/null || true
                $SUDO rm -f /etc/systemd/system/rctruck.service
                $SUDO systemctl daemon-reload
                echo "autostart off (the container keeps running until ./scripts/truck.sh stop)" ;;
            *) echo "usage: $0 autostart on|off"; exit 2 ;;
        esac ;;
    *) sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
