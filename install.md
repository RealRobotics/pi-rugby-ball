# Install and run

## Installation

Start with the latest Raspberry Pi OS Lite (32-bit) released 2023-12-05.

Set up user as "ball" and use UK keyboard and language.

Connect to a hotspot name "BallWiFi" with the correct password.

Use `sudo raspi-config` to enable the I2C bus, the SPI bus and SSH.

Check that the camera works using `rpicam-hello`.  It works even with no GUI and much faster!.

Then install `git` so that you can clone this repo and run this script.

```bash
sudo apt update
sudo apt install git
mkdir ~/git
cd git
git clone https://github.com/RealRobotics/pi-rugby-ball.git
cd pi-rugby-ball
./install.bash
```

The install script installs the python libraries that are needed and clones a few repos that are useful for testing.

## Verifying the installation

There are several test scripts in the directory `test`.  Run each one to test a single sensor.

```bash
cd tests
./camera_video.py # THIS does not work.  Try timed stills
./flow_sensor.py
./imu.py
```

## Running

Run the script `run_all.py`.  Once it settles down,

The magnetometer needs to be calibrated by "rotating the sensor (x-axis upwards) through 360 degrees".  Not sure how that is going to work on the ball.

The IMU

## Camera options

Record bare video with timestamps (in milliseconds).

```bash
rpicam-vid -o test.h264 --save-pts timestamps.txt
```

and then if you want an mkv file (do this on a laptop afterwards):

```bash
mkvmerge -o test.mkv --timecodes 0:timestamps.txt test.h264
```