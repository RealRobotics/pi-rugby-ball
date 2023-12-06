#!/usr/bin/python3
# Copied from
# https://github.com/raspberrypi/picamera2/blob/main/examples/capture_video.py

import time

from picamera2 import Picamera2
from picamera2.encoders import H264Encoder

picam2 = Picamera2()
# video_config = picam2.create_video_configuration()
# picam2.configure(video_config)
video_config_1080 = picam2.video_configuration({"size": (1920, 1080)}, raw={"size": (1640, 1232)}, controls={"FrameDurationLimits": (33333, 33333)})
picam2.configure(video_config_1080)

encoder = H264Encoder(10000000)

picam2.start_recording(encoder, 'test.h264')
time.sleep(10)
picam2.stop_recording()
