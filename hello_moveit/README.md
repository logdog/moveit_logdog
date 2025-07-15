## How to move the UR3e Robot using MoveIt

In all terminals:
```
source ~/ws/install/setup.bash
source ~/ws_moveit/install/setup.bash
source ~/workspace/ros_ur_driver/install/setup.bash
```

In terminal 1, establish communication with the robot:
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.1.102 launch_rviz:=false headless_mode:=true
```

In terminal 2, launch the moveit config for the ur3e:
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
```

In terminal 3, run your C++ or Python MoveIt code:
```
ros2 run hello_moveit hello_moveit
```