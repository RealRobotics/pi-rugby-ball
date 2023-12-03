# The Raspberry Pi Rugby Ball Explorer

6 RPi Zero 2 Ws mounted on a rugby ball shaped device for exploring shafts.  Contains setup notes and scripts to run the code.

To get the RPi ready and to run the code, please follow [these instructions](install/README.md).

## Hardware and drivers

* Flow sensor - PWM3091
  * <https://github.com/RealRobotics/optical_flow_ros>
  * <https://github.com/pimoroni/pmw3901-python>
    * <https://pypi.org/project/spidev/>
    * <https://pypi.org/project/RPi.GPIO/>
* IMU - ICM20948
  * <https://github.com/pimoroni/icm20948-python>
  * <https://github.com/RealRobotics/icm20948-ros2>
  * <https://github.com/CCNYRoboticsLab/imu_tools>
* Camera - ArduCam 12MP IMX708
  * libcamera drivers and tools
    * <https://github.com/anholt/libepoxy>
    * <https://github.com/raspberrypi/libpisp>
    * <https://github.com/raspberrypi/libcamera>
    * <https://github.com/raspberrypi/rpicam-apps>
  * <https://github.com/pipebots/camera_ros>

The wiring schedule is [here](wiring.md).

## Acknowledgments

&copy; 2023, University of Leeds.

The author, A. Blight, has asserted his moral rights.
