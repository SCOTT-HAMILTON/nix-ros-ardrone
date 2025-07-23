# nix-ros-ardrone

This is a nix-based AR Drone 2.0 (Parrot) setup

Too launch de driver you need to rune `roscore`
```shell_session
$ nix develop
$ roscore
```

In another terminal, launch the driver (here we use a 200Hz data transmission rate)
```shell_session
$ nix develop
$ rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=True _navdata_demo:=0 -ip 192.168.1.1
```

In yet another terminal, launch the teleop hotasx controller
```shell_session
$ nix develop
$ roslaunch hotasx_teleop.launch
```

Now, joystick script
```shell_session
$ nix develop
$ python joy.py
```

Now plotjuggler
```shell_session
$ rosrun plotjuggler plotjuggler
```

Or foxglove
```shell_session
$ roslaunch --screen foxglove_bridge launch/foxglove_bridge.launch port:=8765
```

Front image viewer
```shell_session
$ rosrun rqt_image_view rqt_image_view /ardrone/front/image_raw
```

Bottom image viewer
```shell_session
$ rosrun rqt_image_view rqt_image_view /ardrone/bottom/image_raw
```

You can now control the AR Drone with you joystick and access all data provided by the drone

## Camera calibration

For front camera
```shell_session
$ nix develop
$ rosrun camera_calibration cameracalibrator.py --pattern=chessboard --square=0.0748 --size=8x6 image:=/ardrone/front/image_raw
```

For bottom camera
```shell_session
$ nix develop
$ rosrun camera_calibration cameracalibrator.py --pattern=chessboard --square=0.0748 --size=8x6 image:=/ardrone/bottom/image_raw
```

## All-in-one

```shell_session
$ roslaunch nix_ros_ardrone ardrone_setup.launch
```

## Run ORB-SLAM2 Mapping
To start the mapping
```shell_session
$ roslaunch nix_ros_ardrone orb_slam2_ardrone_map.launch
```
And to save the current map to a file (here /home/scott/GIT/nix-ros-ardrone/mono_map.bin)
```shell_session
$ rosservice call /orb_slam2_mono/save_map "/home/scott/GIT/nix-ros-ardrone/mono_map.bin"
```

## Run ORB-SLAM2 Localization
```shell_session
$ roslaunch nix_ros_ardrone orb_slam2_ardrone_localize.launch
```

The Pose object is published to /orb_slam2_mono/pose
