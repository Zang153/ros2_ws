# ROS2 Workspace

This workspace contains the `dynamixel_control` package and requires the `vrpn_mocap` package for motion capture integration.

## Installation

1.  **Clone this repository**:

    ```bash
    git clone https://github.com/Zang153/ros2_ws.git
    cd ros2_ws
    ```

2.  **Clone the `vrpn_mocap` dependency**:

    Since `vrpn_mocap` is ignored in this repository (to avoid nested git repos), you must clone it manually into the `src` directory:

    ```bash
    cd src
    git clone https://github.com/alvinsunyixiao/vrpn_mocap.git
    cd ..
    ```

3.  **Install dependencies**:

    ```bash
    rosdep install --from-paths src -y --ignore-src
    ```

4.  **Build the workspace**:

    ```bash
    colcon build
    ```

5.  **Source the setup file**:

    ```bash
    source install/setup.bash
    ```

## Usage

### Launch VRPN Client and Display Node

To connect to a VRPN server (e.g., OptiTrack) and display the pose data:

```bash
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.31.101 port:=3883
```