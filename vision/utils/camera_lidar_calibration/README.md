# Camera Lidar Calibration

This package handles the camera calibration and camera-lidar calibration.

**Camera-LiDAR calibration is performed in two steps:**

    Obtain camera intrinsics
    Obtain camera-LiDAR extrinsics



## Obtain Camera Intrinsics Calibration

Run by

```
ros2 run camera_lidar_calibration cameracalibrator --size 7x9 --square 0.015 --ros-args --remap image:=/image_raw
```
or launch by

``` 
ros2 launch camera_lidar_calibration cameraCalibration.launch.py
```

## Obtain camera-LiDAR extrinsics 

In works