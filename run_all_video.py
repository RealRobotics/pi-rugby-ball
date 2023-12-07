#!/usr/bin/env python3

import csv
import datetime
import os
import time
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from icm20948 import ICM20948
from pmw3901 import PMW3901, BG_CS_FRONT_BCM

# 0, 90, 180, 270
FLOW_SENSOR_ROTATION = 0
# Data collection frequency in Hertz.
DATA_RATE_HZ = 5.0

class Runner:
    def __init__(self):
        self._imu = ICM20948()
        print("IMU started")
        # Calculate the time to sleep between captures based on the frame rate
        self._sleep_time = 1.0 / DATA_RATE_HZ
        self._image_count = 0

    def _file_init(self):
        self._data_directory = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        try:
            os.mkdir(self._data_directory)
        except FileExistsError:
            pass  # Ignore
        filename = f"{self._data_directory}/data.csv"
        print(f"Writing data to {filename}")
        self._csv_file = open(filename, "w", newline='')
        self._csv_writer = csv.writer(self._csv_file, delimiter=',',
                            quotechar='"', quoting=csv.QUOTE_MINIMAL)
        # Header row
        self._csv_writer.writerow(["Timestamp",
                                   "Acc x", "Acc y", "Acc z",
                                   "Gyr x", "Gyr y", "Gyr z",
                                   "Mag x", "Mag y", "Mag z",
                                   "Flow x", "Flow y",
                                   "FloT x", "FloT y"])

    # Initialize Flow Sensor
    def _flow_sensor_init(self):
        SensorClass = PMW3901
        spi_cs_gpio = BG_CS_FRONT_BCM
        self._flow_sensor = SensorClass(spi_port=0, spi_cs_gpio=spi_cs_gpio)
        self._flow_sensor.set_rotation(FLOW_SENSOR_ROTATION)
        self._total_x = 0.0
        self._total_y = 0.0
        print("Flow sensor started")

    # Initialize Camera
    def _camera_start(self):
        self._camera = Picamera2()
        main_res = {"size": (1920, 1280)}
        configuration = self._camera.create_video_configuration(main=main_res)
        self._camera.configure(configuration)
        self._encoder = H264Encoder(10000000)
        self._camera.start_recording(self._encoder,
                                     f"{self._data_directory}/video.h264",
                                     pts=f"{self._data_directory}/pts.h264")
        print(f"Video recording started using {main_res}")

    def _get_imu_acc_gyro_data(self):
        ax, ay, az, gx, gy, gz = self._imu.read_accelerometer_gyro_data()
        acc_x = f"{ax:05.2f}"
        acc_y = f"{ay:05.2f}"
        acc_z = f"{az:05.2f}"
        gyr_x = f"{gx:05.2f}"
        gyr_y = f"{gy:05.2f}"
        gyr_z = f"{gz:05.2f}"
        return acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z

    def _get_imu_mag_data(self):
        x, y, z = self._imu.read_magnetometer_data()
        mag_x = f"{x:05.2f}"
        mag_y = f"{y:05.2f}"
        mag_z = f"{z:05.2f}"
        return mag_x, mag_y, mag_z

    def _get_flow_sensor_data(self):
        flo_x = ""
        flo_y = ""
        flo_t_x = ""
        flo_t_y = ""
        # Get data from the flow sensor.
        try:
            x, y = self._flow_sensor.get_motion()
            self._total_x += x
            self._total_y += y
            # Convert to strings.
            flo_x = f"{x:05.2f}"
            flo_y = f"{y:05.2f}"
            flo_t_x = f"{self._total_x:05.2f}"
            flo_t_y = f"{self._total_y:05.2f}"
        except RuntimeError:
            pass
        return flo_x, flo_y, flo_t_x, flo_t_y

    def _log_data(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S.%f")
        # Get IMU data.
        acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z = self._get_imu_acc_gyro_data()
        mag_x, mag_y, mag_z = self._get_imu_mag_data()
        # Get flow sensor data.
        flo_x, flo_y, flo_t_x, flo_t_y = self._get_flow_sensor_data()
        # Write to file.
        self._csv_writer.writerow([ timestamp,
                acc_x, acc_y, acc_z,
                gyr_x, gyr_y, gyr_z,
                mag_x, mag_y, mag_z,
                flo_x, flo_y, flo_t_x, flo_t_y,
            ])

    def run(self):
        # Must be first to initialize the data directory.
        self._file_init()
        self._flow_sensor_init()
        self._camera_start()
        try:
            while True:
                self._log_data()
                time.sleep(self._sleep_time)
        except KeyboardInterrupt:
            self._camera.stop()
            self._csv_file.close()


def main(args=None):
    runner = Runner()
    runner.run()

if __name__ == "__main__":
    main()
