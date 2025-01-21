#!/bin/bash

# Exit immediately if a command exits with a non-zero status,
# Treat unset variables as an error, and ensure pipelines fail correctly.
set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_usage() {
  cat <<EOF
Usage: bash $(basename "$0") [OPTIONS]

Options:
  -c, --create                           Create a new GCS container
  -b, --build                            Run the build script
  --delete                               Delete the GCS container
  --mission                              Launch GCS mission_control
  --rviz                                 Launch RViz
  --bash                                 Start bash in the GCS container
  --cmd <command>                        Run a command in the GCS container
  --rlogs <robot_ssh_name>               Get docker logs for the robot
  --rcmd <robot_ssh_name> <command>      Run a command on the robot
  --rdcmd <robot_ssh_name> <command>     Run a command in a Docker container on the robot

  -h, --help                             Display this help message

  === Deprecated ===
  --origin                               Launch GCS mission_origin
  --pac                                  Run PAC status script
  --rqt                                  Launch rqt
  --lpac                                 Run LPAC

EOF

}

# Function to check if a command exists
command_exists() {
  command -v "$1" >/dev/null 2>&1
}

# Function to handle errors
error_exit() {
  echo -e "${RED}Error: $1${NC}" >&2
  exit 1
}
#
# Function to display informational messages
info_message() {
  echo -e "${BLUE}$1${NC}"
}

# Ensure required commands are available
for cmd in docker getopt; do
  if ! command_exists "$cmd"; then
    error_exit "'$cmd' command is not found. Please install it before running this script."
  fi
done

if [[ $# -eq 0 ]]; then
  print_usage
  error_exit "No arguments provided."
fi

# Define short and long options
SHORT_OPTS="cbh"
LONG_OPTS="create,delete,build,bash,cmd:,rcmd:,rdcmd:,mission,origin,pac,rqt,rviz,lpac,rlogs:,help"

# echo "Input arguments: $@"
# # Parse options using getopt
# PARSED_PARAMS=$(getopt --options "$SHORT_OPTS" \
#                        --long "$LONG_OPTS" \
#                        --name "$(basename "$0")" -- "$@") || {
#   error_exit "Failed to parse script options."
# }

# # Evaluate the parsed options
# eval set -- "$PARSED_PARAMS"
# echo "Input arguments: $@"

if [[ -z "${PAC_WS:-}" ]]; then
  error_exit "PAC_WS environment variable is not set. Please set it before running this script."
fi

if [[ ! -d "$PAC_WS" ]]; then
  error_exit "PAC_WS points to '$PAC_WS', but it is not a valid directory."
fi

GPU_FLAG=""

CONTAINER_NAME="gcs"
ROBOT_CONTAINER_NAME="pac-m0054"

docker_cmd() {
  if [[ -z "${1:-}" ]]; then
    error_exit "Missing command to run."
  fi
  info_message "Running command '$1' in container '${CONTAINER_NAME}'..."
  docker exec -it "${CONTAINER_NAME}" bash -ci "$*"
}

robot_cmd() {
  if [[ -z "${1:-}" || -z "${2:-}" ]]; then
    error_exit "Missing robot SSH name or command. Usage: robot_cmd <robot_ssh_name> <command>"
  fi
  SSH_NAME="${1}"
  shift
  info_message "Running command '$*' on '$SSH_NAME'..."
  ssh -t "${SSH_NAME}" "$*"
}

robot_docker_cmd() {
  if [[ -z "${1:-}" || -z "${2:-}" ]]; then
    error_exit "Missing robot SSH name or command. Usage: robot_docker_cmd <robot_ssh_name> <command>"
  fi
  SSH_NAME="${1}"
  shift
  info_message "Running command '$2' in container '${ROBOT_CONTAINER_NAME}' on '$1'..."
  ssh -t "${SSH_NAME}" "docker exec -it ${ROBOT_CONTAINER_NAME} bash -ci '$*'"
}

case "$1" in
  -c|--create)
    info_message "Creating PAC container..."
    if [ "$(command -v nvidia-smi)" ]; then
      GPU_FLAG="--gpu"
    fi
    bash ${PAC_WS}/pac_ws_setup/pac_create_container.sh -d "${PAC_WS}" --ns ${CONTAINER_NAME} -n ${CONTAINER_NAME} --jazzy ${GPU_FLAG}
    ;;
  --delete)
    info_message "Deleting PAC container..."
    docker stop "${CONTAINER_NAME}"
    docker rm "${CONTAINER_NAME}"
    ;;
  -b|--build)
    info_message "Running build script..."
    docker_cmd "/workspace/pac_ws_setup/gcs_build.bash"
    ;;
  --bash)
    docker_cmd "bash"
    ;;
  --cmd)
    shift
    docker_cmd "$*"
    ;;
  --mission)
    info_message "Launching GCS mission_control..."
    docker_cmd "pip install pyqt5"
    xhost +
    docker_cmd "rm -f /root/.config/ros.org/rqt_gui.ini"
    docker_cmd "export DISPLAY='$DISPLAY'; ros2 launch /workspace/launch/mission_control.py"
    ;;
  --rviz)
    info_message "Launching RViz..."
    xhost +
    docker_cmd "export DISPLAY='$DISPLAY'; ros2 launch launch/rviz.yaml"
    ;;
  --rlogs)
    if [[ -z "${1:-}" ]]; then
      error_exit "Missing robot SSH name. Usage: rlogs <robot_ssh_name>"
    fi
    info_message "Getting docker logs from '$2'..."
    robot_cmd "${2}" "docker logs -f pac-m0054"
    ;;
  --rcmd)
    shift
    robot_cmd "$@"
    ;;
  --rdcmd)
    shift
    robot_docker_cmd "$@"
    ;;
  --origin)
    info_message "Launching GCS origin..."
    docker_cmd "ros2 launch /workspace/launch/gcs_origin.yaml"
    ;;
  --pac)
    info_message "Running PAC status script..."
    docker_cmd "ros2 run gcs status_pac"
    ;;
  --rqt)
    info_message "Launching rqt..."
    xhost +
    docker_cmd "export DISPLAY='$DISPLAY'; rqt"
    ;;
  --lpac)
    info_message "Running LPAC status script..."
    docker_cmd "ros2 launch launch/lpac.yaml"
    ;;
  -h|--help)
    print_usage
    exit 0
    ;;
  --)
    break
    ;;
  *)
    info_message "Internal error: unexpected option '$1'" >&2
    print_usage
    ;;
esac
