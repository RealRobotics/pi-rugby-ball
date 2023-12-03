#!/bin/bash
# Install and setup Raspberry Pi specific packages and tools.

# Stop on first error.
set -e

# Tell the user what is going on.
echo
echo "Running $0..."
echo

# Get the directory this script is in.
install_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"

${install_dir}/install_ros2_bare.bash

# Install python packages.
sudo apt install -y build-essential
sudo pip3 install RPi.GPIO spidev pmw3901 icm20948

echo
echo "$0 took $SECONDS seconds."
echo
