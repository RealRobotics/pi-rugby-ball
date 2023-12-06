# Install and run

## Installation

Start with the latest Raspberry Pi OS (64-bit) released 2023-12-05.

Set up user as "ball" and use UK keyboard and language.

Connect to a hotspot name "Ball" with the correct password.

Use `sudo raspi-config` to enable the I2C bus, the SPI bus and SSH.

Then run the following commands:

```bash
sudo pip3 install --break-system-packages icm2094 pmw3901
mkdir ~/git
cd git
git clone https://github.com/pimoroni/pmw3901-python
git clone https://github.com/pimoroni/icm20948-python

```