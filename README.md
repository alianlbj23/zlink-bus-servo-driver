# HiWonder HTD-45H Bus Servo ROS 2 Driver

This project provides a ROS 2 package and standalone Python scripts for controlling HiWonder HTD-45H bus servos connected via a serial interface (e.g., ZLink USB adapter).

## Features

*   **ROS 2 Node (`bus_servo_node`)**:
    *   Located in the `bus_servo_pkg` package.
    *   Subscribes to `trajectory_msgs/msg/JointTrajectoryPoint` messages on the `robot_arm` topic.
    *   Controls multiple servos based on received joint positions (expects **radians**).
    *   Loads configuration (serial port, baud rate, servo IDs, angle limits) from `config/arm_config.yaml`.
*   **Standalone Scripts**:
    *   [`servo_drive.py`](servo_drive.py): Basic script to scan for connected servos, move individual or multiple servos to specific angles (**degrees**), and read their current positions. Useful for testing and direct control.
    *   [`servo_ID.py`](servo_ID.py): Utility script to change the bus ID of a connected servo. **Use with caution: connect only one servo when changing its ID.**
*   **Docker Support**:
    *   [`activate.sh`](activate.sh): Script to easily launch a Docker container
    (`ghcr.io/screamlab/pros_car_docker_image:latest`) with the necessary environment. It specifically looks for the USB device `/dev/usb_robot_arm` on the host system and, if found, passes it into the container using the `--device flag`, enabling direct control of the connected hardware from within the container.

## Hardware
*   **Servo Motor**: [HiWonder HTD-45H Bus Servo](https://www.hiwonder.com/products/htd-45h)
*   **USB-to-Serial Adapter**: A compatible adapter like the [ZLink USB Adapter](https://www.taobao.com/list/item/897427564086.htm) (or similar FTDI/CH340 based adapters) to connect the servos to the computer's USB port.

## Usage
### 1. Using Standalone Scripts

These scripts are run directly with Python and are useful for testing, setup, and direct control without ROS. They use the `PORT` and `BAUD` constants defined within the scripts themselves by default, but can often be overridden with command-line arguments.

*   **Servo Discovery and Drive Test (`servo_drive.py`)**:
    This Python script serves two main purposes:
    1.  **Discovering Servo IDs**: It scans the serial bus (defaulting to IDs 1-30 on `/dev/ttyUSB0`) to find and report the IDs of all connected and responding servos. This is essential for knowing the `<current_id>` needed for other operations.
    2.  **Basic Control**: It allows moving individual or multiple servos to specific angles (**degrees**) and reading their current positions, useful for testing servo functionality.

    To discover connected servo IDs, simply run:
    ```bash
    python3 servo_drive.py
    ```
    The script will print the list of detected IDs. You can modify the script's constants (`PORT`, `BAUD`, `id_range`) or its `if __name__ == "__main__":` block for different tests or scan ranges.

*   **Set Servo ID (`servo_ID.py`)**:
    Once you know the current ID of a servo (using `servo_drive.py`), you can use this utility script to change its bus ID. **Use with caution: Only connect one servo at a time to the bus when changing its ID to avoid conflicts.** The script tries multiple known protocols to set the ID.

    **Usage:**
    ```bash
    python3 servo_ID.py <current_id> <new_id> [--port /dev/ttyUSBX] [--baud 115200]
    ```
    *   `<current_id>`: The current bus ID of the servo (as discovered by `servo_drive.py`).
    *   `<new_id>`: The desired new bus ID for the servo (must be between 1 and 253).
    *   `--port` (optional): Specify the serial port device file (defaults to `/dev/ttyUSB0`).
    *   `--baud` (optional): Specify the serial baud rate (defaults to 115200).

    **Example:** Change a servo currently at ID 1 (found using `servo_drive.py`) to ID 5, connected via `/dev/ttyUSB0`:
    ```bash
    python3 servo_ID.py 1 5 --port /dev/ttyUSB0
    ```
    The script will report if the ID change was successful by checking if the servo responds to the new ID.

### 2. Running the ROS 2 Node

This method uses the ROS 2 framework for controlling the servos, typically integrated into a larger robotic system.

**Steps:**

1.  **Enter the Docker Environment:**
    Use the provided script to start the Docker container which has ROS 2 and necessary dependencies pre-installed. Make sure the script is executable (`chmod +x activate.sh`).
    ```bash
    ./activate.sh
    ```
    This command will launch the container and drop you into a bash shell inside it. The project's `src` directory is mounted at `/workspaces/src`.

2.  **Build the ROS 2 Package (inside Docker):**
    Navigate to the workspace root (which should be `/workspaces` inside the container if you mounted `src` correctly) and build the `bus_servo_pkg`:
    ```bash
    # Inside the Docker container
    cd /workspaces
    colcon build --packages-select bus_servo_pkg
    ```

3.  **Source the Workspace (inside Docker):**
    After building, source the setup file to make the package available in the ROS 2 environment:
    ```bash
    # Inside the Docker container
    source install/setup.bash
    ```

4.  **Launch the Node (inside Docker):**
    Now you can run the servo controller node:
    ```bash
    ros2 run bus_servo_pkg bus_servo_node
    ```
    The node will initialize using the settings from `arm_config.yaml` and start listening for commands on the `/robot_arm` topic.

**Configuration (`config/arm_config.yaml`)**

The behavior of the ROS 2 node is configured through the [`src/bus_servo_pkg/config/arm_config.yaml`](src/bus_servo_pkg/config/arm_config.yaml) file. This file is automatically loaded by the node when it starts.

```yaml
# filepath: src/bus_servo_pkg/config/arm_config.yaml
servo_controller:
  ros__parameters:
    port: /dev/usb_robot_arm      # Serial port device file (must match the device passed to Docker or available on host)
    baud: 115200                  # Baud rate (must match adapter/servo setting)
    speed: 150                    # Default servo movement speed (0-1000) used by the node

    # Configuration for each servo axis controlled by the ROS node
    # The order here MUST match the order of joints in the JointTrajectoryPoint message
    servo_configs:
      - id: 1                     # Servo bus ID
        min: 0.0                  # Minimum operational angle (degrees) for this servo
        max: 240.0                # Maximum operational angle (degrees) for this servo
      - id: 2
        min: 0.0
        max: 240.0
      # Add more servos as needed, maintaining the correct order
