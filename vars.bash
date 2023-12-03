# Variables for other scripts.

# Change this if you want to build your code elsewhere on the host.
WORKSPACE_DIR=${HOME}/ball_ws

# Needed during builds.
UBUNTU_RELEASE=jammy
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
ROS2_DISTRO=${ROS_DISTRO}

# Colcon workspace directories.
COLCON_WS_DIR=${WORKSPACE_DIR}
COLCON_SRC_DIR=${COLCON_WS_DIR}/src
