#!/bin/bash
# Install the 6.2 kernel.

# Stop on first error.
set -e

# Back up the /etc/apt/sources.list so we can restore it later.
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak

# Append the lunar sources to the end of the file.
sudo tee /etc/apt/sources.list >> /dev/null <<'TXT'
# adding this to get the new 6.2.x kernel from lunar
deb http://ports.ubuntu.com/ubuntu-ports lunar main restricted
deb http://ports.ubuntu.com/ubuntu-ports lunar-updates main restricted
deb http://ports.ubuntu.com/ubuntu-ports lunar universe
deb http://ports.ubuntu.com/ubuntu-ports lunar-updates universe
deb http://ports.ubuntu.com/ubuntu-ports lunar multiverse
deb http://ports.ubuntu.com/ubuntu-ports lunar-updates multiverse
deb http://ports.ubuntu.com/ubuntu-ports lunar-backports main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports lunar-security main restricted
deb http://ports.ubuntu.com/ubuntu-ports lunar-security universe
deb http://ports.ubuntu.com/ubuntu-ports lunar-security multiverse
TXT

# Update and install the new kernel.
sudo apt update
sudo apt install linux-image-6.2.0-1017-raspi linux-raspi-headers-6.2.0-1017 linux-modules-6.2.0-1017-raspi linux-raspi-tools-6.2.0-1017
```

# Revert sources.list to the original
sudo cp /etc/apt/sources.list.bak /etc/apt/sources.list

# Reboot for the new kernel to take effect.
reboot