#!/bin/bash
# Build packages, for use after after cloning the repos.

# Include the vars.bash script.
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
setup_dir=${script_dir}
. ${setup_dir}/../vars.bash

echo "ROS_PYTHON_VERSION: ${ROS_PYTHON_VERSION}"
echo "ROS_DISTRO: ${ROS_DISTRO}"

#install dependencies for packages
ROS_PYTHON_VERSION=${ROS_PYTHON_VERSION} rosdep update

echo "Installing dependencies..."
ROS_PYTHON_VERSION=${ROS_PYTHON_VERSION} ROS_DISTRO=$ROS_DISTO rosdep install --from-paths ${WORKSPACE_DIR}/src -y -r --ignore-src

# Build the packages.
echo
echo "Building packages..."
echo
cd ${WORKSPACE_DIR}
. /opt/ros/${ROS2_DISTRO}/setup.bash
colcon build --packages-skip \
    imu_complementary_filter \
    rviz_imu_plugin \
    imu_tools

echo
echo "$0 took $SECONDS seconds."
echo
echo "Source the workspace once the packages have been built:"
echo ". ./install/setup.bash"
echo
