# Pen Detection
My solution for the NU MSR Hackathon [Pen Challenge](https://nu-msr.github.io/hackathon/pen_challenge.html).

## Prerequisites
* ROS 2 Installed

## Installation
```sh
mkdir -p pen_ws/src
cd pen_ws/src
git clone git@github.com:0nhc/pen_detection.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Demo
```sh
# Make sure you have connected your Realsense 400 Series Camera
cd pen_ws
source install/setup.bash
ros2 launch pen_detection main.launch.py
```

