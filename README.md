# The Raspberry Pi Rugby Ball Explorer

6 RPi Zero 2 Ws mounted on a rugby ball shaped device for exploring shafts.  Contains setup notes and scripts to run the code.

To get the RPi ready and to run the code, please follow [these instructions](install.md).

The RPi Zero 2W are not the best at handling large video images and proved to be quite limited as far as performance goes.  Details can be found in the [Camera Test Results](camera_test_results.md).

TLDR;
Maximum reliable video size: 1920x1280 at 30fps.
Maximum still frame rate at 4608x2592 = 1fps, 2304x1296 = 2fps, 1536x864 = 3fps.
Writes to micro SD card are main blockage.

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
