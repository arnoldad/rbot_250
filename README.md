

build it
```
catkin_make
```

launch it
```
 roslaunch haas_vf2 gazebo.launch
```

run a motion plan
```
rosservice call /vf2_motion "x_pos: 25.0
y_pos: 25.0"
```

reset the thing
```
rosservice call /vf2_position "x_pos: 0.0
y_pos: 0.0"
```