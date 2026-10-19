# iRED ROS 2 Workspace

ROS 2 Jazzy workspace for iRED robot development, navigation, and testing on Ubuntu 24.04.

## Project Structure

- `src/` - ROS 2 packages
- `docker/client/` - Client-side ROS 2 environment
- `docker/rpi5/` - Raspberry Pi 5 emulator for testing on your computer
- `build.sh` - Builds the workspace
- `setup_rpi5.sh` - Sets up the Raspberry Pi 5 environment
- `tmux-navi.sh` - Starts navigation
- `tmux-test.sh` - Starts individual nodes for testing

## Setup

Run the setup script **once** on the intended Ubuntu 24.04 system. The script installs ROS 2 Jazzy dependencies and configures the system.

```bash
./setup_rpi5.sh [ROS_DOMAIN_ID]
```

`ROS_DOMAIN_ID` is optional and defaults to `102`.

Examples:

```bash
./setup_rpi5.sh       # Use default domain ID 102
./setup_rpi5.sh 10    # Use domain ID 10
```

The script already prevents repeated setup, so you do not need to run it again after a successful setup.

## Build

Build the workspace:

- After cloning the repository for the first time
- After adding or modifying ROS 2 packages or source files
- Whenever package dependencies change

```bash
source /opt/ros/jazzy/setup.bash
./build.sh
source install/setup.bash
```

You do not need to rebuild if you have not changed anything since the last successful build.

## Docker Environments

Each directory contains `Dockerfile`, `compose.yaml`, and scripts to install, run, and stop its environment.

- `docker/client/` - Client-side ROS 2 development and visualization.
- `docker/rpi5/` - Raspberry Pi 5 emulator for testing without the physical robot.

## Running

Start navigation:

```bash
./tmux-navi.sh
```

Start individual nodes for testing:

```bash
./tmux-test.sh
```

## Emulator Testing

The emulator supports testing without physical hardware. Hardware-dependent nodes may require simulated inputs.

For navigation testing, verify that `/map`, `/scan`, `/scan_filtered`, odometry, and the required TF transforms are available before troubleshooting Nav2.
