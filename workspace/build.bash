#!/bin/bash
# Build packages, for use after after cloning the repos.
# Must be run from the workspace directory.

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
