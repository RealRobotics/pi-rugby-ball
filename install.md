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
git clone https://github.com/raspberrypi/picamera2
```

## Verifying the installation

There are several test scripts in the directory `test`.  Run each one to test a single sensor.

```bash
cd tests
./camera_video.py
./flow_sensor.py
./imu.py
```

## Running

The magnetometer needs to be calibrated by "rotating the sensor (x-axis upwards) through 360 degrees".

The IMU