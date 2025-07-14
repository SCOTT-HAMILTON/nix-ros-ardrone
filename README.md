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

Now plotjuggler
```shell_session
$ rosrun plotjuggler plotjuggler
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
