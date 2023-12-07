#!/usr/bin/python3
# Based on the example
# https://github.com/raspberrypi/picamera2/blob/main/examples/capture_timelapse.py

import time

from picamera2 import Picamera2

FRAME_RATE = 0.5
MAX_FRAMES = 10

picam2 = Picamera2()
# main_res = {"size": (4608, 2592)}
main_res = {"size": (2304, 1296)}
# main_res = {"size": (1536, 864)}
# main_res = {"size": (800, 600)}
configuration = picam2.create_still_configuration(main=main_res)
picam2.configure(configuration)
picam2.start()

# Give time for Aec and Awb to settle, before disabling them
time.sleep(1)
picam2.set_controls({"AeEnable": False, "AwbEnable": False, "FrameRate": FRAME_RATE})
print(f"Set frame rate to {FRAME_RATE} for {MAX_FRAMES} frames using {main_res}")
# And wait for those settings to take effect
time.sleep(1)

start_time = time.time()
for i in range(1, MAX_FRAMES + 1):
    r = picam2.capture_request()
    print(f"Captured image {i} of {MAX_FRAMES} at {time.time() - start_time:.2f}s")
    r.save("main", f"image{i}.jpg")
    r.release()
    print(f"Wrote image {i} of {MAX_FRAMES} at {time.time() - start_time:.2f}s")

total_time = time.time() - start_time
picam2.stop()

effective_frame_rate = MAX_FRAMES / total_time
print(f"Total time taken {total_time:.2f}s")
print(f"Effective frame rate {effective_frame_rate}")
