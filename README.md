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

You can now control the AR Drone with you joystick
