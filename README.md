# coprocessor-FRC-2020

manual exposure setting:
http://www.peterklemperer.com/blog/2018/02/10/manual-exposure-control-of-opencv-video/

show all cameras:
ls -ltrh /dev/video*

documentation of odom table:  
note: all coordinate system are converted to WPI format, x+ forward of robot, y+ leftward of robot, righthand rule (0 deg forward, turn left increase theta) 
* "t265_pose_[xyzrpt]": the 3d pose with respect to t265's starting pose
* "field_pose_[xyt]": the 2d pose of the robot with respect to the field, this is only usable after field_calibration
* "target_field_azm": in field coordinate, the angle the robot should turn to face the target, only use if "target_found" is false
* "target_found": whether the target is found by the RGB camera
* "field_calibration_start": set this flag to true if the robot is static and facing the correct target, usually at the start of the game. After recognizing the flag, Nano will set this to false.
* "field_calibration_good": whether the field_calibration is done. This should be set to false after Nano disconnects.
* "target_t265_azm": if "target_found" is true, this shows the theta that the robot should turn to face the target, with respect to t265. This is lag-compensated and should be used when possible.
* "target_dis": the distance from camera to target in 2d plane.
* "target_relative_dir_right": in case t265 disconnects but RGB camera works, this shows how many degrees right the robot should turn to face the target
* "error_xy" / "error_theta": when "target_found" is true, these two fields show the error between the field pose return by camera and by t265 using last field calibration. This is not lag-compensated. 
