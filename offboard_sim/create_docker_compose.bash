#!/bin/bash

# Check if an argument is provided
if [ -z "$1" ]; then
  echo "Usage: $0 <num_robots>"
  exit 1
fi

NUM_ROBOTS=$1
OUTPUT_FILE="docker-compose.yaml"

# Write the common settings to the docker-compose file
cat <<EOL > $OUTPUT_FILE
x-common-settings: &common-settings
  image: agarwalsaurav/pac:jazzy
  privileged: true
  network_mode: host
  ipc: host
  pid: host
  environment:
    - RCUTILS_COLORIZED_OUTPUT=1
    - PAC_WS=/workspace
  working_dir: /workspace
  volumes:
    - \${PAC_WS}:/workspace:rw

x-ros-launch-file: &ros-launch-file
  command: >
    bash -c -i "ros2 launch /workspace/launch/starling_offboard.yaml"

services:
EOL

# Generate services for each robot
for ((i=1; i<=NUM_ROBOTS; i++)); do
  cat <<EOL >> $OUTPUT_FILE
  px4_$i:
    <<: [*common-settings, *ros-launch-file]
    container_name: px4_$i
    environment:
      - ROS_NAMESPACE=px4_$i
      - ROBOT_ID=$i

EOL
done

echo "docker-compose.yaml generated with $NUM_ROBOTS robots."
