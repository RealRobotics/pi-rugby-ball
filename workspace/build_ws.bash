#!/bin/bash
# Build packages, for use after after cloning the repos.

# Include the vars.bash script.
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
setup_dir=${script_dir}
. ${setup_dir}/../vars.bash


echo "ROS_PYTHON_VERSION: ${ROS_PYTHON_VERSION}"
echo "ROS_DISTRO: ${ROS_DISTRO}"

# Install dependencies for ROS packages
echo
echo "Updating dependencies..."
. /opt/ros/${ROS_DISTRO}/setup.bash
rosdep init
rosdep update --rosdistro ${ROS_DISTRO}
echo
echo "Installing dependencies..."
rosdep install --from-paths ${WORKSPACE_DIR}/src -y -r --ignore-src --rosdistro ${ROS_DISTRO}

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
