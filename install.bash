#! /bin/bash
# Install the examples and a couple of tools.

# Install necessary tools.
sudo apt update
sudo apt install python3-pip
sudo pip3 install --break-system-packages icm20948 pmw3901

# Install the examples.
cd git
git clone https://github.com/pimoroni/pmw3901-python.git
git clone https://github.com/pimoroni/icm20948-python.git
git clone https://github.com/raspberrypi/picamera2.git

# This repo makes using git easier from the command line.
git clone https://github.com/andyblight/bash_scripts.git
cd bash_scripts
./install.sh ubuntu22.04lts

echo
echo "Tools and examples installed."
echo "Exit this shell and start a new one to use the new tools."
echo
