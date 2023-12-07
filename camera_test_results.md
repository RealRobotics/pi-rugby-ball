# Camera test results

The camera is capable of very hi-resolution images.  The RPi Zero 2W is designed for small embedded systems not image processing, so there are a few limitations.

## Still images

The script `camera_timed_stills.py` can be used to show the limitations by changing the frame rate and the resolution.

The following output shows the speed limitations of the camera as used on the Pi Zero 2W at maximum resolution, 4608x2592-BGR888.

```text
Set frame rate to 10.0 for 10 frames using {'size': (4608, 2592)}
Captured image 1 of 10 at 0.05s
Wrote image 1 of 10 at 0.93s
Captured image 2 of 10 at 1.14s
Wrote image 2 of 10 at 1.92s
Captured image 3 of 10 at 2.14s
Wrote image 3 of 10 at 2.92s
Captured image 4 of 10 at 3.14s
Wrote image 4 of 10 at 3.92s
Captured image 5 of 10 at 4.14s
Wrote image 5 of 10 at 4.92s
Captured image 6 of 10 at 5.14s
Wrote image 6 of 10 at 5.92s
Captured image 7 of 10 at 6.14s
Wrote image 7 of 10 at 6.92s
Captured image 8 of 10 at 7.14s
Wrote image 8 of 10 at 7.92s
Captured image 9 of 10 at 8.14s
Wrote image 9 of 10 at 8.92s
Captured image 10 of 10 at 9.14s
Wrote image 10 of 10 at 9.92s
Total time taken 9.92s
Effective frame rate 1.0079376425480153
```

In this case, the image capture time is about 200ms and the image write to the micro SD card takes about 800ms (average files size around 800kB). The effective maximum frame rate at maximum resolution is 1fps.

Same again for medium resolution.

```text
Set frame rate to 10.0 for 10 frames using {'size': (2304, 1296)}
Captured image 1 of 10 at 0.14s
Wrote image 1 of 10 at 0.43s
Captured image 2 of 10 at 0.63s
Wrote image 2 of 10 at 0.84s
Captured image 3 of 10 at 1.03s
Wrote image 3 of 10 at 1.24s
Captured image 4 of 10 at 1.43s
Wrote image 4 of 10 at 1.64s
Captured image 5 of 10 at 1.83s
Wrote image 5 of 10 at 2.04s
Captured image 6 of 10 at 2.23s
Wrote image 6 of 10 at 2.46s
Captured image 7 of 10 at 2.63s
Wrote image 7 of 10 at 2.84s
Captured image 8 of 10 at 3.03s
Wrote image 8 of 10 at 3.24s
Captured image 9 of 10 at 3.43s
Wrote image 9 of 10 at 3.64s
Captured image 10 of 10 at 3.83s
Wrote image 10 of 10 at 4.06s
Total time taken 4.06s
Effective frame rate 2.4615315273863976
```

This time, the capture time is still around 200ms but the file write time has come down to about 200ms (average file size around 400kB).

Same again for lowish resolution.

```text
Set frame rate to 10.0 for 10 frames using {'size': (1536, 864)}
Captured image 1 of 10 at 0.12s
Wrote image 1 of 10 at 0.31s
Captured image 2 of 10 at 0.52s
Wrote image 2 of 10 at 0.62s
Captured image 3 of 10 at 0.82s
Wrote image 3 of 10 at 0.92s
Captured image 4 of 10 at 1.12s
Wrote image 4 of 10 at 1.25s
Captured image 5 of 10 at 1.42s
Wrote image 5 of 10 at 1.52s
Captured image 6 of 10 at 1.72s
Wrote image 6 of 10 at 1.82s
Captured image 7 of 10 at 2.02s
Wrote image 7 of 10 at 2.16s
Captured image 8 of 10 at 2.32s
Wrote image 8 of 10 at 2.42s
Captured image 9 of 10 at 2.62s
Wrote image 9 of 10 at 2.72s
Captured image 10 of 10 at 2.92s
Wrote image 10 of 10 at 3.04s
Total time taken 3.04s
Effective frame rate 3.2924447715543343
```

This time, the capture time is still around 200ms but the file write time has come down to about 100ms (average file size around 166kB).

Same again for 800x600.

```text
Set frame rate to 10.0 for 10 frames using {'size': (800, 600)}
Captured image 1 of 10 at 0.04s
Wrote image 1 of 10 at 0.15s
Captured image 2 of 10 at 0.34s
Wrote image 2 of 10 at 0.38s
Captured image 3 of 10 at 0.54s
Wrote image 3 of 10 at 0.58s
Captured image 4 of 10 at 0.74s
Wrote image 4 of 10 at 0.78s
Captured image 5 of 10 at 0.94s
Wrote image 5 of 10 at 0.99s
Captured image 6 of 10 at 1.14s
Wrote image 6 of 10 at 1.18s
Captured image 7 of 10 at 1.34s
Wrote image 7 of 10 at 1.38s
Captured image 8 of 10 at 1.54s
Wrote image 8 of 10 at 1.58s
Captured image 9 of 10 at 1.74s
Wrote image 9 of 10 at 1.79s
Captured image 10 of 10 at 1.94s
Wrote image 10 of 10 at 2.00s
Total time taken 2.00s
Effective frame rate 4.995719881202436
```

This time, the capture time is still around 200ms but the file write time has come down to about 50ms (average file size around 63kB).

Clearly, there is a limitation in the software that limits the capture time to about 200ms.

Same again for 800x600 at a requested 20fps.

```text
Set frame rate to 20.0 for 10 frames using {'size': (800, 600)}
Captured image 1 of 10 at 0.07s
Wrote image 1 of 10 at 0.18s
Captured image 2 of 10 at 0.27s
Wrote image 2 of 10 at 0.31s
Captured image 3 of 10 at 0.42s
Wrote image 3 of 10 at 0.46s
Captured image 4 of 10 at 0.57s
Wrote image 4 of 10 at 0.61s
Captured image 5 of 10 at 0.72s
Wrote image 5 of 10 at 0.76s
Captured image 6 of 10 at 0.87s
Wrote image 6 of 10 at 0.92s
Captured image 7 of 10 at 1.02s
Wrote image 7 of 10 at 1.06s
Captured image 8 of 10 at 1.17s
Wrote image 8 of 10 at 1.21s
Captured image 9 of 10 at 1.32s
Wrote image 9 of 10 at 1.36s
Captured image 10 of 10 at 1.47s
Wrote image 10 of 10 at 1.51s
Total time taken 1.51s
Effective frame rate 6.639511124253386
```

The capture time has come down to around 110ms, so it appears that the frame rate value is out by an amount.

```text
Set frame rate to 40.0 for 10 frames using {'size': (800, 600)}
Captured image 1 of 10 at 0.05s
Wrote image 1 of 10 at 0.15s
Captured image 2 of 10 at 0.19s
Wrote image 2 of 10 at 0.23s
Captured image 3 of 10 at 0.29s
Wrote image 3 of 10 at 0.33s
Captured image 4 of 10 at 0.39s
Wrote image 4 of 10 at 0.43s
Captured image 5 of 10 at 0.49s
Wrote image 5 of 10 at 0.53s
Captured image 6 of 10 at 0.59s
Wrote image 6 of 10 at 0.63s
Captured image 7 of 10 at 0.69s
Wrote image 7 of 10 at 0.73s
Captured image 8 of 10 at 0.79s
Wrote image 8 of 10 at 0.84s
Captured image 9 of 10 at 0.90s
Wrote image 9 of 10 at 0.94s
Captured image 10 of 10 at 1.00s
Wrote image 10 of 10 at 1.04s
Total time taken 1.04s
Effective frame rate 9.602315744023656
```

This time the capture time is around 60ms, much faster but not the 25ms requested.

Now to got to max. resolution again:

```text
Set frame rate to 40.0 for 10 frames using {'size': (4608, 2592)}
Captured image 1 of 10 at 0.16s
Wrote image 1 of 10 at 1.07s
Captured image 2 of 10 at 1.40s
Wrote image 2 of 10 at 2.19s
Captured image 3 of 10 at 2.52s
Wrote image 3 of 10 at 3.30s
Captured image 4 of 10 at 3.63s
Wrote image 4 of 10 at 4.42s
Captured image 5 of 10 at 4.75s
Wrote image 5 of 10 at 5.53s
Captured image 6 of 10 at 5.86s
Wrote image 6 of 10 at 6.65s
Captured image 7 of 10 at 6.98s
Wrote image 7 of 10 at 7.76s
Captured image 8 of 10 at 8.09s
Wrote image 8 of 10 at 8.88s
Captured image 9 of 10 at 9.21s
Wrote image 9 of 10 at 9.99s
Captured image 10 of 10 at 10.32s
Wrote image 10 of 10 at 11.11s
Total time taken 11.11s
Effective frame rate 0.9002632746529714
```

Image capture time is around 300ms which is worse than at a requested rate of 10fps.  Very interesting but not useful!

Without a lot more experimentation, I'm not sure if this is the Pi Zero being slow, a bug in the code or both.

## Video

The program `camera_video.py` was used to test the capabilities of the Pi Zero.  Video was tried at various resolutions using the default encoding of h264.

* 1280x720 - Video worked well.
* 1920x1080 - Video worked well.
* 3840x2160 - DMA buffer allocation failed.  Lack of memory.
