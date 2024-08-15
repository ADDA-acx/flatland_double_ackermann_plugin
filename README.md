# Double Ackermann robot simulation plug-in for flatland robot simulator

**This plugin performs Ackermann and Double Ackermann kinematics modeling in flatland simulator by bicycle modeling**

**Video**：https://www.bilibili.com/video/BV1RU3yeME8Z/?spm_id_from=333.999.0.0&vd_source=7321921c0512a33755eb173d7b59ddea

## Usage

### Dependency

- **Ubuntu: 20.04**
- **ROS: Notic**

### Setup

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git https://github.com/ADDA-acx/flatland_double_ackermann_plugin.git
cd ..
rosdep install --from-paths src --ignore-src #For installing dependencies
catkin_make
source devel/setup.bash
roslaunch double_ackermann_flatland double_ackermann_in_flatland.launch 
```

### Keyboard control

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Reference

https://github.com/avidbots/flatland

