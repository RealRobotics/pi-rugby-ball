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
${install_dir}/arducam/install_arducam.bash

echo
echo "Please configure the RPi Camera options and set up the PiGPIO crontab entry."
echo "Then reboot to ensure the correct permissions are given for GPIO access."
echo
echo "$0 took $SECONDS seconds."
echo
