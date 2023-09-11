# NTU_Racing_Hw
written by Chih Han Yang

The repo directory structure is shown as following:
```
ntu_racing_hw
├── docker
│   └── packages
│       └── ros2
│           ├── hw_ws
│           │   └── src
│           │       ├── ros2_homework1
│           │       └── ros2_homework2
│           ├── launch_ws
│           └── ros2_ws
├── docs
│   ├── html
│   │   └── search
│   └── latex
├── git
├── ros2_setup
│   ├── Dockerfile
│   ├── packages
│   │   └── ros2
│   └── pictures
├── .clang-format
├── README.md
└── ROS2_Tutorial_Review.md
```
1. ROS2 Homework 1 is to collect the code from the tutorial. It's inside the directory:
```
ntu_racing_hw/docker/packages/ros2/launch_ws
ntu_racing_hw/docker/packages/ros2/ros2_ws
```
2. ROS2 Homework 2 is here:
```
ntu_racing_hw/docker/packages/ros2/hw_ws/src/ros2_homework2
```
3. The doxygen documentation are inside the docs directory:
```
ntu_racing_hw/docs
```
4. The git homework is inside the directory:
```
ntu_racing_hw/git
```
5. The directory that contains docker files, and all the build tools running the docker container. (the ```Makefile``` is for easy command representation.)
```
ntu_racing_hw/ros2_setup
```
6. Last but not least, I recorded all my reviews on the tutorial in:
```
ROS2_Tutorial_Review.md
```

Hope you enjoy it ~