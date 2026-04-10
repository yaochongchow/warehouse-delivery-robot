#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_DIR="${ROOT_DIR}/.docker"
SERVICE_NAME="ros2"

DASHBOARD_URL="http://localhost:8080/"
NOVNC_URL="http://localhost:6080/"

if [[ ! -f "${COMPOSE_DIR}/docker-compose.yaml" ]]; then
  echo "Error: ${COMPOSE_DIR}/docker-compose.yaml not found."
  exit 1
fi

DOCKER_CMD=()
if docker info >/dev/null 2>&1; then
  DOCKER_CMD=(docker)
elif command -v sudo >/dev/null 2>&1; then
  if sudo -n docker info >/dev/null 2>&1; then
    DOCKER_CMD=(sudo -n docker)
  else
    DOCKER_CMD=(sudo docker)
  fi
else
  echo "Error: cannot access Docker daemon and sudo is unavailable."
  exit 1
fi

echo "==> Building and starting Docker service"
(
  cd "${COMPOSE_DIR}"
  "${DOCKER_CMD[@]}" compose up --build -d --remove-orphans
)

CONTAINER_ID="$(
  cd "${COMPOSE_DIR}" &&
  "${DOCKER_CMD[@]}" compose ps -q "${SERVICE_NAME}"
)"

if [[ -z "${CONTAINER_ID}" ]]; then
  echo "Error: could not find running container for service '${SERVICE_NAME}'."
  exit 1
fi

echo "==> Starting ROS launches in container ${CONTAINER_ID}"
"${DOCKER_CMD[@]}" exec "${CONTAINER_ID}" bash -lc '
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
set -u

kill_pid_file() {
  local pid_file="$1"
  if [[ -f "${pid_file}" ]]; then
    local old_pid
    old_pid="$(cat "${pid_file}" 2>/dev/null || true)"
    if [[ -n "${old_pid}" ]]; then
      kill "${old_pid}" 2>/dev/null || true
      sleep 0.2
      if kill -0 "${old_pid}" 2>/dev/null; then
        kill -9 "${old_pid}" 2>/dev/null || true
      fi
    fi
    rm -f "${pid_file}"
  fi
}

kill_by_pattern() {
  local pattern="$1"
  local signal_name="${2:-TERM}"
  if command -v pgrep >/dev/null 2>&1; then
    while read -r pid; do
      [[ -n "${pid}" ]] || continue
      [[ "${pid}" = "$$" ]] && continue
      kill -"${signal_name}" "${pid}" 2>/dev/null || true
    done < <(pgrep -f "${pattern}" || true)
  fi
}

force_kill_by_pattern() {
  local pattern="$1"
  kill_by_pattern "${pattern}" TERM
  sleep 0.5
  kill_by_pattern "${pattern}" KILL
}

kill_pid_file /tmp/full_system_live.pid
kill_pid_file /tmp/web_dashboard_live.pid

force_kill_by_pattern "ros2 launch warehouse_bringup full_system.launch.py"
force_kill_by_pattern "ros2 launch warehouse_dashboard web_dashboard.launch.py"
force_kill_by_pattern "rosbridge_websocket"
force_kill_by_pattern "python3 -m http.server 8080"
force_kill_by_pattern "spawn_entity.py"
force_kill_by_pattern "robot_state_publisher"
force_kill_by_pattern "(^|/)gzserver( |$)"
force_kill_by_pattern "(^|/)gzclient( |$)"

sleep 1

nohup bash -lc "set +u; source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u; ros2 launch warehouse_bringup full_system.launch.py" >/tmp/full_system_live.log 2>&1 &
echo $! >/tmp/full_system_live.pid

nohup bash -lc "set +u; source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u; ros2 launch warehouse_dashboard web_dashboard.launch.py" >/tmp/web_dashboard_live.log 2>&1 &
echo $! >/tmp/web_dashboard_live.pid

# Wait until order service appears (max ~180s)
for _ in $(seq 1 90); do
  if ros2 service list 2>/dev/null | grep -q "^/submit_order$"; then
    break
  fi
  sleep 2
done

echo "full_system_pid=$(cat /tmp/full_system_live.pid)"
echo "web_dashboard_pid=$(cat /tmp/web_dashboard_live.pid)"
'

wait_for_url() {
  local url="$1"
  local max_tries="$2"
  local i

  if ! command -v curl >/dev/null 2>&1; then
    return 0
  fi

  for i in $(seq 1 "${max_tries}"); do
    if curl -fsS "${url}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

open_url() {
  local url="$1"
  if command -v xdg-open >/dev/null 2>&1; then
    xdg-open "${url}" >/dev/null 2>&1 &
  elif command -v gio >/dev/null 2>&1; then
    gio open "${url}" >/dev/null 2>&1 &
  fi
}

echo "==> Waiting for dashboard endpoint"
if ! wait_for_url "${DASHBOARD_URL}" 60; then
  echo "Warning: ${DASHBOARD_URL} not reachable yet."
fi

open_url "${DASHBOARD_URL}"

echo
echo "Project is running."
echo "- Dashboard: ${DASHBOARD_URL}"
echo "- noVNC:     ${NOVNC_URL}"
echo
echo "Useful commands:"
echo "- Full system log: ${DOCKER_CMD[*]} exec ${CONTAINER_ID} tail -f /tmp/full_system_live.log"
echo "- Web log:         ${DOCKER_CMD[*]} exec ${CONTAINER_ID} tail -f /tmp/web_dashboard_live.log"
echo "- Stop launches:   ${DOCKER_CMD[*]} exec ${CONTAINER_ID} bash -lc 'kill \$(cat /tmp/full_system_live.pid /tmp/web_dashboard_live.pid 2>/dev/null)'"
