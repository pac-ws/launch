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
  --bash                                 Start bash in the GCS container
  --bash_cmd <command>                   Run a command in the GCS container
  --mission                              Launch GCS mission_control
  --origin                               Launch GCS mission_origin
  --pac                                  Run PAC status script
  --rqt                                  Launch rqt
  --rviz                                 Launch RViz
  --lpac                                 Run LPAC
  --rlogs <robot_ssh_name>               Get docker logs for the robot

  -h, --help                             Display this help message

Examples:
  bash $(basename "$0") --create
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
  exit 1
fi

# Define short and long options
SHORT_OPTS="cbh"
LONG_OPTS="create,delete,build,bash,bash_cmd:,mission,origin,pac,rqt,rviz,lpac,rlogs:,help"

# Parse options using getopt
PARSED_PARAMS=$(getopt --options "$SHORT_OPTS" --long "$LONG_OPTS" --name "$(basename "$0")" -- "$@") || {
  echo "Failed to parse arguments." >&2
  print_usage
  exit 1
}

# Evaluate the parsed options
eval set -- "$PARSED_PARAMS"

if [[ -z "${PAC_WS:-}" ]]; then
  error_exit "PAC_WS environment variable is not set. Please set it before running this script."
fi

if [[ ! -d "$PAC_WS" ]]; then
  error_exit "PAC_WS points to '$PAC_WS', but it is not a valid directory."
fi

GPU_FLAG=""
CONTAINER_NAME="gcs"
while true; do
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
      docker stop ${CONTAINER_NAME}
      docker rm ${CONTAINER_NAME}
      ;;
    -b|--build)
      info_message "Running build script..."
      docker exec -it ${CONTAINER_NAME} bash -ci pac_ws_setup/gcs_build.bash
      ;;
    --bash)
      info_message "Starting bash in Docker container..."
      docker exec -it ${CONTAINER_NAME} bash
      ;;
    --bash_cmd)
      if [[ -z "${2:-}" || "$2" =~ ^-+ ]]; then
        error_exit "Missing command to run. Usage: --bash_cmd <command>"
      fi
      info_message "Running command "$2" in Docker container..."
      docker exec -it ${CONTAINER_NAME} bash -ci "$2"
      shift
      ;;
    --mission)
      info_message "Launching GCS mission_control..."
      xhost +
      docker exec -it ${CONTAINER_NAME} bash -ci "export DISPLAY=:0; ros2 launch launch/mission_control.yaml"
      ;;
    --origin)
      info_message "Launching GCS origin..."
      docker exec -it ${CONTAINER_NAME} bash -ci "ros2 launch launch/gcs_origin.yaml"
      ;;
    --pac)
      info_message "Running PAC status script..."
      docker exec -it ${CONTAINER_NAME} bash -ci "ros2 run gcs status_pac"
      ;;
    --rqt)
      info_message "Launching rqt..."
      xhost +
      docker exec -it ${CONTAINER_NAME} bash -ci "export DISPLAY=:0; rqt"
      ;;
    --rviz)
      info_message "Launching RViz..."
      xhost +
      docker exec -it ${CONTAINER_NAME} bash -ci "export DISPLAY=:0; ros2 launch launch/rviz.yaml"
      ;;
    --lpac)
      info_message "Running LPAC status script..."
      docker exec -it ${CONTAINER_NAME} bash -ci "ros2 launch launch/lpac.yaml"
      ;;
    --rlogs)
      if [[ -z "${2:-}" || "$2" =~ ^-+ ]]; then
        error_exit "Missing robot SSH name. Usage: --rlogs <robot_ssh_name>"
      fi
      ROBOT_SSH_ID="$2"
      info_message "Getting docker logs from '$ROBOT_SSH_ID'..."
      ssh -t "${ROBOT_SSH_ID}" "docker logs -f pac-m0054"
      shift
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    *)
      info_message "Internal error: unexpected option '$1'" >&2
      print_usage
      ;;
  esac
  shift
done

