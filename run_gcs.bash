#!/bin/bash

# Enable error handling
set -e

# Function to show usage
usage() {
  echo "Usage: $0 [--create|--delete|--build|--bash|--origin|--pac|--rviz|--lpac]"
  exit 1
}

# Check if any argument is provided
if [ $# -eq 0 ]; then
  usage
fi

GPU_FLAG=""
# Check if nvidia gpu is available
# Parse the provided flags
while [ "$#" -gt 0 ]; do
  case "$1" in
    --create)
      echo "Creating PAC container..."
      if [ "$(command -v nvidia-smi)" ]; then
        GPU_FLAG="--gpu"
      fi
      bash ${PAC_WS}/pac_ws_setup/pac_create_container.sh -d "${PAC_WS}" --ns gcs -n gcs --noble ${GPU_FLAG}
      ;;
    --delete)
      echo "Deleting PAC container..."
      docker stop gcs
      docker rm gcs
      ;;
    --build)
      echo "Running build script..."
      docker exec -it gcs bash -ci pac_ws_setup/gcs_build.bash
      ;;
    --bash)
      echo "Starting bash in Docker container..."
      docker exec -it gcs bash
      ;;
    --origin)
      echo "Launching GCS origin..."
      docker exec -it gcs bash -ci "ros2 launch launch/gcs_origin.yaml"
      ;;
    --pac)
      echo "Running PAC status script..."
      docker exec -it gcs bash -ci "ros2 run gcs status_pac"
      ;;
    --rviz)
      echo "Launching RViz..."
      xhost +
      docker exec -it gcs bash -ci "export DISPLAY=:0; ros2 launch launch/rviz.yaml"
      ;;
    --lpac)
      echo "Running LPAC status script..."
      docker exec -it gcs bash -ci "ros2 launch launch/lpac.yaml"
      ;;
    *)
      echo "Error: Invalid option '$1'"
      usage
      ;;
  esac
  shift
done

