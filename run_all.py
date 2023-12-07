#!/usr/bin/env python3

import picamera
from picamera import cameras
import time
import datetime
from pimoroni_icm20948 import ICM20948

# Initialize Camera
camera = picamera.PiCamera(cameras.PiCamera())

# Set camera resolution and frame rate (adjust as needed)
camera.resolution = (640, 480)
camera.framerate = 30

# Initialize IMU
imu = ICM20948()

# Initialize Flow Sensor (replace YourFlowSensorClass with the actual class)
flow_sensor = YourFlowSensorClass()

# Calculate the time to sleep between captures based on the frame rate
sleep_time = 1 / camera.framerate

def log_data():
    timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")

    # Capture image from the camera
    camera.capture(f'img_{timestamp}.jpg')

    # Get IMU data
    imu_data = imu.read_accel_data(), imu.read_gyro_data(), imu.read_magnet_data()

    # Get flow sensor data
    flow_data = flow_sensor.get_data()

    # Log data to a file or database
    with open(f'data_{timestamp}.txt', 'w') as file:
        file.write(f"Timestamp: {timestamp}\n")
        file.write("IMU Data:\n")
        file.write(f"Acceleration: {imu_data[0]}\n")
        file.write(f"Gyroscope: {imu_data[1]}\n")
        file.write(f"Magnetometer: {imu_data[2]}\n")
        file.write("Flow Sensor Data:\n")
        file.write(str(flow_data) + "\n")

# Start recording with the specified frame rate
camera.start_recording('/dev/null', format='h264', intra_period=2)

try:
    while True:
        log_data()
        camera.wait_recording(sleep_time)

except KeyboardInterrupt:
    # Stop recording and close the camera on keyboard interrupt
    camera.stop_recording()
    camera.close()
