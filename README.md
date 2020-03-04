# coprocessor-FRC-2020

This is a python project running on a Jetson Nano doing computer vision and localization for FIRST Robotics Competition 2020.  
The project uses three cameras:
1. A Microsoft LifeCam HD-3000 to detect the reflective tape on the shooting target
2. An Intel Realsense D435 for ball detection using RGBD data
3. An Intel Realsense T265 for visual odometry  

-------------------------------
### Dependencies
1. [OpenCV 4.2.0](https://github.com/JetsonHacksNano/buildOpenCV)
2. [Intel Realsense SDK (with pyrealsense2)](https://github.com/JetsonHacksNano/installLibrealsense)
3. [pynetworktables](https://robotpy.readthedocs.io/en/stable/install/index.html)
4. [robotpy-cscore](https://robotpy.readthedocs.io/en/stable/install/cscore.html) (optional, for CameraServer video streaming)

-------------------------------

## documentation of odom table:  
##### note: 
* all coordinate system are converted to WPI format, x+ forward of robot, y+ leftward of robot, righthand rule (0 deg forward, turn left increase theta)
* the poses of RGB camera and t265 with respect to robot need to be set in PoseTracker
 
##### status flags:
* `pose_good`: whether t265 is working
* `target_good`: whether RGB camera working
* `ball_good`: whether D435 is working
* `client_time`: time update indicating that nano is alive and connected

##### localization:
* `robot_[xyt]`: the 2d pose with respect to t265's starting pose
* `field_calibration_start`: set this flag to true if the robot is static and facing the correct target, usually at the start of the game. After recognizing the flag, Nano will set this to false.
* `field_calibration_good`: whether the field_calibration is done. This should be set to false after Nano disconnects.
* `field_[xyt]`: the 2d pose of the robot with respect to the field, this is only usable after field_calibration
* `error_xy` / `error_theta`: when `target_found` is true, these two fields show the error between the field pose return by camera and by t265 using last field calibration. This is not lag-compensated.
* `encoder_v`: this is the input for wheel odometry data, which needs to be set up in config

##### shooting target detection:
* `target_found`: whether the target is found by the RGB camera
* `target_field_theta`: in field coordinate, the angle the robot should turn to have RGb camera align with the target
* `target_dis`: the distance from camera to target in 2d plane.
* `target_relative_dir_left`: in case t265 disconnects but RGB camera works, this shows how many degrees left the robot should turn to face the target
* `CV/camera_{xyztpr}`: the raw camera x, y, z, yaw, pitch, roll data in world from solvepnp

##### ball detection (similar to target detection)
`ball_found`:  
`ball_dis`:  
`ball_to_left`:  
`ball_field_theta`:

--------------------------------------

### manual exposure setting:
v4l2-ctl -d /dev/video0 --list-ctrls  
v4l2-ctl -d /dev/video0 -c auto_exposure=1  
http://www.peterklemperer.com/blog/2018/02/10/manual-exposure-control-of-opencv-video/

### show all cameras:
ls -ltrh /dev/video*