## PalmBee

PalmBee is a package designed to automate the pollination process in palm tree plantations using drones. This package leverages the capabilities of PX4 and ROS2 Humble to provide an efficient and reliable solution for pollination.

## Compatibility

- **Ubuntu 22.04**
- **ROS2 Humble**
- **PX4**

## Installation

To install the PalmBee package, follow these steps:

1. Create the workspace directory:
    ```bash
    mkdir -p palmbee_ws/src
    cd palmbee_ws/src
    ```

2. Clone the repository:
    ```bash
    git clone https://github.com/ep51lon/PalmBee.git
    ```

3. Install dependencies:
    ```bash
    cd palmbee_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. Build the package:
    ```bash
    colcon build
    ```
    or build specific package (for example, pb_perception) using
    ```bash
    colcon build --packages-select pb_perception
    ```

5. Source the setup file:
    ```bash
    source install/setup.bash
    ```

## Usage

**TODO**

To use the PalmBee package, follow these steps:

1. Launch the drone simulation:
    ```bash
    ros2 launch palmbee drone_simulation.launch.py
    ```

2. Start the pollination process:
    ```bash
    ros2 run palmbee pollinate.py
    ```

For more detailed instructions and documentation, please refer to the [wiki](https://github.com/yourusername/palmbee/wiki).

## Contributing

We welcome contributions to the PalmBee package. Please fork the repository and submit pull requests for any enhancements or bug fixes.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.