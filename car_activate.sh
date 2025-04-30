#!/bin/bash

# --- Configuration ---
# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Define paths relative to the script directory
# Assuming devices.txt and esp32_reset.py are directly in SCRIPT_DIR
DEVICES_TXT_FILE="${SCRIPT_DIR}/devices.txt"
ESP32_RESET_PY="${SCRIPT_DIR}/esp32_reset.py"
# Assuming src, setting_script are subdirectories, and .env is in SCRIPT_DIR
ARM_SRC_DIR="${SCRIPT_DIR}/src"
ARM_SETTING_SCRIPT_DIR="${SCRIPT_DIR}/setting_script"
ARM_ENV_FILE="${SCRIPT_DIR}/.env"

# Docker network name
DOCKER_NETWORK="compose_my_bridge_network"

# Container names
ARM_CONTAINER_NAME="arm_controller_node"
AGENT_CONTAINER_PREFIX="microros_agent_"

# Images
PROS_CAR_IMAGE="ghcr.io/screamlab/pros_car_docker_image:latest"
# Use the same image for reset as specified in the original agent script if different
ESP32_RESET_IMAGE="registry.screamtrumpet.csie.ncku.edu.tw/alianlbj23/pros_car_docker_image:latest"
MICRO_ROS_AGENT_IMAGE="microros/micro-ros-agent:humble"

# --- Globals ---
declare -a ALL_CONTAINER_NAMES=() # Store names of all managed containers

# --- Cleanup Function ---
cleanup() {
    echo # Newline for cleaner output
    echo "Stopping all managed containers..."
    if [ ${#ALL_CONTAINER_NAMES[@]} -eq 0 ]; then
        echo "No containers were tracked to stop."
    else
        # Stop arm controller first if it exists in the list
        if [[ " ${ALL_CONTAINER_NAMES[@]} " =~ " ${ARM_CONTAINER_NAME} " ]]; then
             echo "Stopping container: $ARM_CONTAINER_NAME"
             docker kill "$ARM_CONTAINER_NAME" > /dev/null 2>&1 || echo "Failed to kill or already stopped: $ARM_CONTAINER_NAME"
             # Remove it from the list to avoid double processing if needed
             temp_list=()
             for name in "${ALL_CONTAINER_NAMES[@]}"; do [[ "$name" != "$ARM_CONTAINER_NAME" ]] && temp_list+=("$name"); done
             ALL_CONTAINER_NAMES=("${temp_list[@]}")
        fi
        # Stop remaining agent containers
        for container_name in "${ALL_CONTAINER_NAMES[@]}"; do
            echo "Stopping container: $container_name"
            docker kill "$container_name" > /dev/null 2>&1 || echo "Failed to kill or already stopped: $container_name"
        done
        # Clear the array after attempting cleanup
        ALL_CONTAINER_NAMES=()
    fi
    echo "Cleanup complete."
    # Remove trap to avoid recursion on exit
    trap - SIGINT SIGTERM EXIT
    exit 0
}

# --- Trap Setup ---
# Call cleanup function on Ctrl+C (SIGINT), termination (SIGTERM), or script exit (EXIT)
trap cleanup SIGINT SIGTERM EXIT

# --- 1. ESP32 Reset ---
echo "--- Running ESP32 Reset ---"
if [ -f "$ESP32_RESET_PY" ] && [ -f "$DEVICES_TXT_FILE" ]; then
    # Mount the files from SCRIPT_DIR into the container's /workspace
    docker run --rm --privileged \
      --network "$DOCKER_NETWORK" \
      -v "${ESP32_RESET_PY}:/workspace/esp32_reset.py" \
      -v "${DEVICES_TXT_FILE}:/workspace/devices.txt" \
      -v /dev:/dev \
      "$ESP32_RESET_IMAGE" \
      bash -c "python3 /workspace/esp32_reset.py"
    echo "ESP32 Reset attempt finished."
else
    echo "Warning: ESP32 reset script or devices file not found in ${SCRIPT_DIR}. Skipping reset."
    echo "Looked for: $ESP32_RESET_PY"
    echo "Looked for: $DEVICES_TXT_FILE"
fi
echo "---------------------------"
sleep 1 # Short pause

# --- 2. Start micro-ROS Agents ---
echo "--- Starting micro-ROS Agents ---"
AGENT_CONTAINER_NAMES_VALID=()
if [ -f "$DEVICES_TXT_FILE" ]; then
    # Read device paths from the text file (skip comment lines)
    DEVICES=()
    RAW_CONTAINER_NAMES=() # Names derived from device paths
    while IFS= read -r line || [ -n "$line" ]; do
      # Skip empty lines and comments
      if [[ -n "$line" && ! "$line" =~ ^[[:space:]]*# ]]; then
        DEVICES+=("$line")
        # Generate container name
        RAW_NAME=$(echo "$line" | sed 's|/dev/||g' | sed 's|/|_|g')
        RAW_CONTAINER_NAMES+=("${AGENT_CONTAINER_PREFIX}${RAW_NAME}")
      fi
    done < "$DEVICES_TXT_FILE"

    # Start agent for each device
    for i in "${!DEVICES[@]}"; do
      DEVICE="${DEVICES[$i]}"
      CONTAINER_NAME="${RAW_CONTAINER_NAMES[$i]}"

      # Check if device exists before starting container
      if [ -e "$DEVICE" ]; then
        echo "Starting micro-ROS agent for ${DEVICE} as ${CONTAINER_NAME}"
        docker run -d --rm \
          --privileged \
          --name "${CONTAINER_NAME}" \
          --network "$DOCKER_NETWORK" \
          -v /dev:/dev \
          -e ROS_DOMAIN_ID=1 \
          "$MICRO_ROS_AGENT_IMAGE" \
          serial --dev "${DEVICE}"

        # Check if container started successfully (basic check)
        if [ $? -eq 0 ]; then
            echo "Agent container ${CONTAINER_NAME} started."
            AGENT_CONTAINER_NAMES_VALID+=("$CONTAINER_NAME")
            ALL_CONTAINER_NAMES+=("$CONTAINER_NAME") # Add to global list
        else
            echo "Error: Failed to start agent container ${CONTAINER_NAME} for ${DEVICE}."
        fi
      else
        echo "Warning: Device ${DEVICE} not found, skipping agent container creation."
      fi
    done
else
    echo "Warning: devices.txt not found at ${DEVICES_TXT_FILE}. Cannot start micro-ROS agents."
fi

if [ ${#AGENT_CONTAINER_NAMES_VALID[@]} -eq 0 ]; then
  echo "No valid micro-ROS agent containers were started."
  # Decide if this is critical - maybe exit? For now, just warn.
fi
echo "--------------------------------"
sleep 1 # Short pause

# --- 3. Start Arm Controller Node ---
echo "--- Starting Arm Controller Node ---"
# Check if required directories/files exist
if [ ! -d "$ARM_SRC_DIR" ]; then
    echo "Error: Arm source directory not found at ${ARM_SRC_DIR}. Cannot start arm controller."
    cleanup # Stop any agents that might have started
    exit 1
fi
if [ ! -d "$ARM_SETTING_SCRIPT_DIR" ]; then
    echo "Warning: Arm setting script directory not found at ${ARM_SETTING_SCRIPT_DIR}."
fi
if [ ! -f "$ARM_ENV_FILE" ]; then
    echo "Warning: .env file not found at ${ARM_ENV_FILE}"
fi


# Define volume mounts relative to SCRIPT_DIR
ARM_VOLUME_ARGS=(
    "-v" "${ARM_SRC_DIR}:/workspaces/src"
    "-v" "${ARM_SETTING_SCRIPT_DIR}:/workspaces/setting_script"
)

# Define Docker run arguments for arm controller
ARM_DOCKER_RUN_ARGS=(
    "run" "-d" "--rm" # Run detached, remove on exit
    "--name" "$ARM_CONTAINER_NAME"
    "--network" "$DOCKER_NETWORK"
)

# Add env file if it exists
if [ -f "$ARM_ENV_FILE" ]; then
    ARM_DOCKER_RUN_ARGS+=("--env-file" "$ARM_ENV_FILE")
fi

# Conditionally add device
if [ -e /dev/usb_robot_arm ]; then
    ARM_DOCKER_RUN_ARGS+=("--device=/dev/usb_robot_arm")
else
    echo "Warning: /dev/usb_robot_arm not found. Arm controller might not work."
fi

# Add volume mounts
ARM_DOCKER_RUN_ARGS+=( "${ARM_VOLUME_ARGS[@]}" )

# Add image and command
# Note: Removed "; bash" from the end for detached execution
ARM_DOCKER_RUN_ARGS+=(
    "$PROS_CAR_IMAGE"
    "bash" "-c" "cd /workspaces && colcon build && . ./install/setup.bash && ros2 run bus_servo_pkg bus_servo_node"
)

# Execute the docker command
echo "Starting container ${ARM_CONTAINER_NAME}..."
docker "${ARM_DOCKER_RUN_ARGS[@]}"

# Check if container started successfully
if [ $? -eq 0 ]; then
    echo "Arm controller container ${ARM_CONTAINER_NAME} started."
    ALL_CONTAINER_NAMES+=("$ARM_CONTAINER_NAME") # Add to global list
else
    echo "Error: Failed to start arm controller container ${ARM_CONTAINER_NAME}."
    # Decide if this is critical
    cleanup # Stop any agents that might have started
    exit 1
fi
echo "-----------------------------------"

# --- 4. Wait Loop ---
if [ ${#ALL_CONTAINER_NAMES[@]} -eq 0 ]; then
    echo "No containers were successfully started. Exiting."
    exit 1
fi

echo # Newline
echo "All managed containers started."
echo "Running containers:"
# List only the containers managed by this script
docker ps --filter "name=${ARM_CONTAINER_NAME}" --filter "name=${AGENT_CONTAINER_PREFIX}" --format "  {{.Names}} (ID: {{.ID}}, Status: {{.Status}})"
echo # Newline
echo "Press 'q' or 'Q' or Ctrl+C to stop all managed containers."

while true; do
    read -rsn1 -t 1 input # Read 1 char, silent, timeout 1s
    if [[ "$input" == "q" || "$input" == "Q" ]]; then
        echo " 'q' pressed, initiating cleanup..."
        cleanup # Call cleanup function
        exit 0 # Exit script
    fi
    # Optional: Check if any managed container stopped unexpectedly
    # active_containers=$(docker ps --filter "name=${ARM_CONTAINER_NAME}" --filter "name=${AGENT_CONTAINER_PREFIX}" --format "{{.Names}}")
    # if [ "$(echo "$active_containers" | wc -l)" -ne "${#ALL_CONTAINER_NAMES[@]}" ]; then
    #     echo "Warning: One or more managed containers may have stopped unexpectedly."
    #     # You could compare the list of active containers with ALL_CONTAINER_NAMES
    #     # and potentially trigger cleanup or other actions.
    # fi

    # The loop continues if no 'q' and no signal received
done

# Should not be reached due to trap and 'q' handling
exit 0