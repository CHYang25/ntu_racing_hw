# ROS2_Tutorial_Review

There's a typo in the docker image file 'ros2_host'

After the configuration of the docker, I started going through the tutorial.

## Beginners: CLI Tools

### Using turtlesim, ros2, and rqt
#### turtlesim
Turtlesim is a simulator illustrating the functionality of ROS2 on a robot, which is helpful for me to learn ROS2. To install it, run this inside the container:
```
sudo apt update
sudo apt install ros-humble-turtlesim
```
(ps, the sudo password should be ```docker``` according to [NTURacingTeam/docker](https://github.com/NTURacingTeam/docker))

The package is installed:
```
docker@ros2:~$ ros2 pkg executables turtlesim
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

Then, start the simulator.
```
ros2 run turtlesim turtlesim_node
```
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-08-13%20085447.png)

We can now use the turtlesim.
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-08-13%20142611.png)

#### rqt
According to ROSWiki, the introduction to rqt is described as following:
> rqt is a software framework of ROS that implements the various GUI tools in the form of plugins. One can run all the existing GUI tools as dockable windows within rqt! The tools can still run in a traditional standalone method, but rqt makes it easier to manage all the various windows on the screen at one moment.

We also install that in our container by ```sudo apt install ~nros-humble-rqt*```.
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-08-13%20095046.png)

Now, I started to use the rqt following the tutorial. Note that, the turtlesim node should be running to reproduce what is shown on the tutorial. The turtlesim is a dockable window that is referred to as a plugin.

Spawn the second turtle <br>
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-08-19%20061022.png)

Set pen of turtle1 <br>
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-08-19%20061503.png)

Remapping the topic <br>
The topic is the edge between nodes. Since by default, the command would build the teleop_key to turtle1. So we need to remap the topic between the teleop_key to turtle2.
```
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```
Turtle2 has moved successfully <br>
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-08-19%20063319.png)

### Nodes
Control a modular function. Nodes can communicate with each other via:
1. topics
2. services
3. actions
4. parameters

This is how to run an executable in ros2
```
ros2 run <package_name> <executable_name>
```
The previous examples have shown that ```turtlesim``` is the package name, and ```trutlesim_node``` is the executable launched.

To check what node is currently running, command ```ros2 node list```. This is the result after launching the turtlesim node and the teleop_key node: 
```
docker@ros2:~$ ros2 node list
/teleop_turtle
/turtlesim
```
To check out the information of one node, command ```ros2 node info <node_name>```. This is the result after launching the turtlesim node and checking out its information.
```
docker@ros2:~$ ros2 node info /turtlesim
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:

```

### Topics
To visualize the graph of the ros2, use ```rqt_graph```. Here's the result:
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-08-19%20090815.png)

To check what node is currently running, command ```ros2 topic list -t```. This is the result after launching the turtlesim node and the teleop_key node: 
```
docker@ros2:~$ ros2 topic list -t
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```

To see the data being published on a topic, command ```ros2 topic echo <topic_name>```. Check out the data on the topic "/turtle1/cmd_vel", and here's the result: (I pressed the right arrow then the up arrow to the teleop_turtle console, so we can tell that the turtle would first recieve an rotation command then a forward-moving command)
```
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -2.0
---
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```

Also we can observe that the echo command is functioning with rqt_graph visualization. <br>
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-08-19%20094311.png)

To check out the topic info, use ```ros2 topic info <topic_name>```. Here's the result:
```
docker@ros2:~ros2 topic info /turtle1/cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 1
```
(Every topic has publishers and subscriptions as inputs and outputs, with nodes at its both ends)

What is the ```Type``` property? It shows that the topic has the type Twist, which is in the msg file inside the geometry_msgs package. The command ```ros2 interface show geometry_msgs/msg/Twist``` gives us
```
# This expresses velocity in free space broken into its linear and angular parts.

    Vector3  linear
            float64 x
            float64 y
            float64 z
    Vector3  angular
            float64 x
            float64 y
            float64 z
```
The word interface could be related to the java term interface.

To publish data onto a topic, use ```ros2 topic pub <topic_name> <msg_type> '<args>'```. The result is:
```
docker@ros2:~$ ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.8))
```
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-08-19%20101303.png)
Instead of using the ```teleop_key``` node to publish controlling data, we use the ```ros2 topic pub``` command node to control that turtle by publishing the arguments to that specific topic.

The ```--once``` option tells the node to publish once then terminate.

We can make the trutle running in a circle with ```ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"```, where ```--rate 1``` option means to publish such data under 1 Hz frequency. <br>
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-08-19%20101806.png)

With the publishing node(/_ros2cli_805), and the echo node(/_ros2cli_877), the graph shoud look like this: <br>
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-08-19%20103133.png)

To view the rate at which data is published, use ```ros2 topic hz <topic_name>```. Recalled that we use the pub command to publish data onto topic ```/turtle1/cmd_vel``` with rate 1. So that the result should look like this: 
```
docker@ros2:~$ ros2 topic hz /turtle1/cmd_vel
average rate: 1.000
        min: 0.999s max: 1.000s std dev: 0.00057s window: 3
average rate: 1.000
        min: 0.999s max: 1.000s std dev: 0.00056s window: 4
average rate: 1.000
        min: 0.999s max: 1.000s std dev: 0.00053s window: 5
.
.
.
```

### Services
Nodes could also communicate through services as well. It's based on a call-and-response model.

To show the service list, use ```ros2 service list```. Here's the result
```
docker@ros2:~$ ros2 service list
/clear
/kill
/reset
/spawn
/teleop_turtle/describe_parameters
/teleop_turtle/get_parameter_types
/teleop_turtle/get_parameters
/teleop_turtle/list_parameters
/teleop_turtle/set_parameters
/teleop_turtle/set_parameters_atomically
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
```
Recall the part about rqt, that service drop list is shown here. (ps, the parameters will be in the next part)

I realized that recording the tutorial step by step is time-consuming. From now on, I'll only collect those worth noting.

### Services
Using ```ros2 interface show <type_name>``` can show the information about that type. The "---" separates the request structure (above) from the response structure (below).
```
ros2 interface show turtlesim/srv/Spawn

// then return

float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

Another example:
```
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```
It can spawn a turtle using the service. But what's the deifferene between such service call and the publishing call? Quoted from the official website:
> Nodes can communicate using services in ROS 2. Unlike a topic - a one way communication pattern where a node publishes information that can be consumed by one or more subscribers - a service is a request/response pattern where a client makes a request to a node providing the service and the service processes the request and generates a response.

![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-08-19%20112939.png)

### Parameters
> A parameter is a configuration value of a node. You can think of parameters as node settings. 

Change the background color of turtle sim:
```
docker@ros2:~$ ros2 param set /turtlesim background_r 150
Set parameter successful
```
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-08-19%20115758.png)

```ros2 param dump <name> > <file>```
> Dumping parameters comes in handy if you want to reload the node with the same parameters in the future.
To load it from the future, use ```ros2 param load <name> <file>```
If you want to load the parameters while starting a node -> ```ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>```

### Actions
> They consist of three parts: a goal, feedback, and a result.
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-08-20%20131104.png)

To check the information of one action, use:
```ros2 action info <action>```

To see the action list of a node, use:
```ros2 action list -t```, -t is for showing the action type.

We also can use interface show to acknowledge the action data type within. 
>The section of this message above the first --- is the structure (data type and name) of the goal request. The next section is the structure of the result. The last section is the structure of the feedback.

To send a goal of an action: ```ros2 action send_goal <action_name> <action_type> <values>```

> A robot system would likely use actions for navigation. An action goal could tell a robot to travel to a position. While the robot navigates to the position, it can send updates along the way (i.e. feedback), and then a final result message once it’s reached its destination.

### rqt_console
>Fatal messages indicate the system is going to terminate to try to protect itself from detriment.

>Error messages indicate significant issues that won’t necessarily damage the system, but are preventing it from functioning properly.

>Warn messages indicate unexpected activity or non-ideal results that might represent a deeper issue, but don’t harm functionality outright.

>Info messages indicate event and status updates that serve as a visual verification that the system is running as expected.

>Debug messages detail the entire step-by-step process of the system execution.

These are the 5 severity levels of messages. The severity filter would filter out those with less severity level than the chosen option.

To set the severity level:
```
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN
```

### Record Topics
To record topic data:
```
ros2 bag record <topic_name>
```
It should create a directory ```rosbag2_year_month_day-hour_minute_second``` with a file ```metadata.yaml``` that records the data.

To record multiple topics, just list them out. -o option is for naming:
```
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose
```

To check the bag info
```
ros2 bag info <bag_file_name>
```

To replay what just recorded, use:
```
ros2 bag play <bag_file_name>
```

>Because the subset file recorded the /turtle1/pose topic, the ros2 bag play command won’t quit for as long as you had turtlesim running, even if you weren’t moving.This is because as long as the /turtlesim node is active, it publishes data on the /turtle1/pose topic at regular intervals. You may have noticed in the ros2 bag info example result above that the /turtle1/cmd_vel topic’s Count information was only 9; that’s how many times we pressed the arrow keys while recording.Notice that /turtle1/pose has a Count value of over 3000; while we were recording, data was published on that topic 3000 times.To get an idea of how often position data is published, you can run the command:

## Beginners: Client Libraries

### Creating a wrokspace

#### Resolve dependencies
>Before building the workspace, you need to resolve the package dependencies. You may have all the dependencies already, but best practice is to check for dependencies every time you clone. You wouldn’t want a build to fail after a long wait only to realize that you have missing dependencies.

To check the dependencies:
```
rosdep install -i --from-path src --rosdistro humble -y
```

>Packages declare their dependencies in the package.xml file

#### Source the overlay
>In the new terminal, source your main ROS 2 environment as the “underlay”, so you can build the overlay “on top of” it:
```
# source the underlay
source /opt/ros/humble/setup.bash
cd ~/ws/src/ros2_ws
# source the overlay
source install/local_setup.bash
```
(The source command is just basically executing the setup file that build the ROS2 environment and the overlay package)

or just run
```
source install/steup.bash
# it sources both ROS2 environment as underlay and the packages as overlay
```

Then run the turtlesim. To tell the concept of overlay and underlay, we should know whether the turtlesim we ran is from the package or from the original installation. To achieve that, we can modify the overlay.

This is the result after modifying the source file and ```colcon build``` it again.

![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-08-26%20205304.png)

> modifications in the overlay did not actually affect anything in the underlay

>In this tutorial, you sourced your main ROS 2 distro install as your underlay, and created an overlay by cloning and building packages in a new workspace. The overlay gets prepended to the path, and takes precedence over the underlay, as you saw with your modified turtlesim. Using overlays is recommended for working on a small number of packages, so you don’t have to put everything in the same workspace and rebuild a huge workspace on every iteration.


### Creating a package
> A package is an organizational unit for your ROS 2 code. 

> Package creation in ROS 2 uses ament as its build system and colcon as its build tool. You can create a package using either CMake or Python, which are officially supported

In a CMake package directory, it should include:
1. ```CMakeLists.txt``` file that describes how to build the code within the package
2. ```include/<package_name>``` directory containing the public headers for the package
3. ```package.xml``` file containing meta information about the package
4. ```src``` directory containing the source code for the package

To create a new package in ros2:
```
ros2 pkg create --build-type ament_cmake <package_name>
```

```colcon build``` can build multiple packages at once, as long as those packages are in the same workspace directory. To build only one package:
```
colcon build --packages-select my_package
```

It's important to run ```source install/local_setup.bash``` to add the workspace to the path, inorder to use the package executables.

We should also customize the package.xml, inside the description tag, maintainer tag, and licenses tag.
>Below the license tag, you will see some tag names ending with _depend. This is where your package.xml would list its dependencies on other packages, for colcon to search for. 

### C++ publisher and subscriber
**Communicate through topics**

To add dependencies, we should let cmake know what headers are needed. To do so, we should modify the ```package.xml``` file and ```CMakeLists.txt``` file, and add dependencies according to the headers included.
```
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
```

```
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```
(Note that the dependencies tags should be added after ament_cmake)

```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```

Also in ```CMakeLists.txt```, add this to run the code using ```ros2 run```:
```
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
```
And add this so that ```ros2 run``` can find the executables:
```
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```
(talker is the executable name)

>The main function is exactly the same, except now it spins the MinimalSubscriber node. For the publisher node, spinning meant starting the timer, but for the subscriber it simply means preparing to receive messages whenever they come.

Then we also wget the subscriber code. Note that we don't need to modify the ```package.xml``` file, for the headers needed are the same. But we should modify the ```CMakeLists.txt``` file, for the cmake should capture the code.
```
.
.
.
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

Then run ```rosdep``` in the root of your workspace (ros2_ws) to check for missing dependencies before building:
```
rosdep install -i --from-path src --rosdistro humble -y
```
Then use colcon to build the new package:
```
colcon build --packages-select cpp_pubsub
```

Also, don't forget to source that bash inorder to find the executables.
```
. install/setup.bash
```

Then run it: (The package name is ```cpp_pubsub```, and the node(executable) name is ```talker```)
```
ros2 run cpp_pubsub talker
```

### C++ Client and Server
**Communicate through services**
> The structure of the request and response is determined by a ```.srv``` file.
The ```.srv``` file in the following example looks like (defining the data type of the request and response):
```
int64 a
int64 b
---
int64 sum
```
(example_interfaces is the package that includes the .srv file)

While creating a package, we can add the dependencies option to create the correct files by default.
```
ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces
```

Inside ```CmakeLists.txt```, note that although dependencies are built by the last command, we still need the ```add_executable()``` macro generates an executable so that we can run it through ```ros2 run```
```
add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)
```
```
install(TARGETS
    server
  DESTINATION lib/${PROJECT_NAME})
```
(ps that ```RCLCPP_INFO``` is like ```fprintf```)

Also, after finishing the client's code, we should modify the ```CMakeLists.txt``` as well. After all, in the root workspace directory, we should check the dependencies by:
```
rosdep install -i --from-path src --rosdistro humble -y
```
Then build it with the following command (don't forget to check the returned message that the colcon build was successful)
```
colcon build --packages-select cpp_srvcli
```
Then, don't forget to source the setup file so that the paths to those executables are added.
```
source install/setup.bash
```

When running, we can notice that the server is continue running no matter there's a client or not. When a client occurs, it recieve the request and return the response after processing. 

### Creating custom msg and srv files
Remember that these types of files define the communication of two nodes in an ros2 graph, aka intefaces. One defines topics, and the other defines services.

Inside ```Sphere.msg```, it uses a message from another message package-```geometry_msgs/Point```.
(In the package geometry_msgs, the msg named Point)

After creating the msg files and srv files, modify ```CMakeLists.txt``` and ```package.xml```.
```
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```
The find_package is understandable. we need the ```geometry_msgs``` package, for it's used in the ```Sphere.msg```. We need the ```rosidl_default_generators```, for we need it to generate language-specific code (c++ in this case) from the msg or srv files. (ps rosidl stands for ros interface definition language)

```
<depend>geometry_msgs</depend> # The dependencies of the package interface it self
<buildtool_depend>rosidl_default_generators</buildtool_depend> # This is the dependencies of the buildtool
<exec_depend>rosidl_default_runtime</exec_depend> # this is the dependencies during runtime to use the interfaces
<member_of_group>rosidl_interface_packages</member_of_group> # this is the the package that my package should be associated with
```
>Because the interfaces rely on rosidl_default_generators for generating language-specific code, you need to declare a build tool dependency on it. rosidl_default_runtime is a runtime or execution-stage dependency, needed to be able to use the interfaces later. The rosidl_interface_packages is the name of the dependency group that your package, tutorial_interfaces, should be associated with, declared using the <member_of_group> tag.

Then colcon build, source the setup. Remember that we've check the interfaces before while we learned about topics. Now, we use the ```ros2 interface show``` command to see whether our interface is available or not.
```
ros2 interface show tutorial_interfaces/msg/Num

# return
int64 num
```
```
ros2 interface show tutorial_interfaces/msg/Sphere

#return
geometry_msgs/Point center
        float64 x
        float64 y
        float64 z
float64 radius
```
```
ros2 interface show tutorial_interfaces/srv/AddThreeInts

#return
int64 a
int64 b
int64 c
---
int64 sum
```
Now we can test whether we could use these interfaces or not. Use the publisher and subscriber code and modify it. Then add a few lines to ```CMakeLists.txt``` and ```package.xml``` files to meet the source code. Afterall, build 
it, then source it. Then in two terminals run the talker and listener separately.

By the way, the tutorial modified the code used in previous ones, but I want to keep the previous version intact so I created the source code inside the ```tutorial_interfaces``` package. However, an error occurred while delivering ```colcon build```:
```
The package "tutorial_interfaces" must not "build_depend" on a package with the same name as this package
The package "tutorial_interfaces" must not "build_export_depend" on a package with the same name as this package  
The package "tutorial_interfaces" must not "exec_depend" on a package with the same name as this package
```
So that I created another package ```cpp_pubsub_interfaces``` to keep each version clean and healthy. By the way, recall that nested packages are not allowed in ROS2. In this case, self-dependencies is also nested by its child is itself. Also, the changes are about the message type (```tutorial_interfaces::msg::Num``` and ```std_msgs::msg::String```). (ps. tutorial_interfaces should be built before cpp_pubsub_tutorial)

ALso, be careful that the project name in the ```CMakeLists.txt``` should also be changed to ```cpp_pubsub_interfaces``` as well. Otherwise, ```$PROJECT_NAME``` won't match.

Finally, source the setup files, then run ```ros2 run cpp_pubsub_interfaces talker``` and ```ros2 run cpp_pubsub_interfaces listener``` in two different terminals.

Do the same to the ```AddThreeInts.srv``` with client and server. Create a new package, and set up the configuration files, then build those files, then finally, source it then run it. By the way, the newly created package, ```cpp_srvcli_interfaces``` also uses the ```tutorial_interfaces``` package. Here's the result
```
docker@ros2:~/ws/src/ros2_ws$ ros2 run cpp_srvcli_interfaces server
[INFO] [1693291703.803293812] [rclcpp]: Ready to add three ints.
[INFO] [1693291714.448700490] [rclcpp]: Incoming request
a: 2 b: 3 c: 1
[INFO] [1693291714.448747191] [rclcpp]: sending back response: [6]
^C[INFO] [1693291717.010731524] [rclcpp]: signal_handler(signum=2)
```
```
docker@ros2:~/ws/src/ros2_ws$ ros2 run cpp_srvcli_interfaces client 2 3 1
[INFO] [1693291714.449219891] [rclcpp]: Sum: 6
```

### Implementing custom interfaces
> Note that it’s possible to set default values for fields within a message definition.
A package named ```more_interfaces``` is created. And these should go to ```package.xml```.
```
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
>Note that at build time, we need ```rosidl_default_generator```s, while at runtime, we only need ```rosidl_default_runtime```.

As for the ```CMakeLists.txt``` file, add:
```
find_package(rosidl_default_generators REQUIRED)

# it just lists the messages out inn msg_files
set(msg_files
  "msg/AddressBook.msg"
)

# transform the idl into c++ cod
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

# export the message runtime dependency
ament_export_dependencies(rosidl_default_runtime)
```
(ps. we can use ```set``` like this to list out all our interfaces)
```
set(msg_files
  "msg/Message1.msg"
  "msg/Message2.msg"
  # etc
  )

set(srv_files
  "srv/Service1.srv"
  "srv/Service2.srv"
   # etc
  )
# generate all at once
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)
```

There's one thing worth noting is that, it uses a lambda expression in the constructor. I don't understand what it is until I searched on Google. [Link](https://learn.microsoft.com/en-us/cpp/cpp/lambda-expressions-in-cpp?view=msvc-170).
```
auto publish_msg = [this]() -> void {
        auto message = more_interfaces::msg::AddressBook();

        message.first_name = "John";
        message.last_name = "Doe";
        message.phone_number = "1234567890";
        message.phone_type = message.PHONE_TYPE_MOBILE;

        std::cout << "Publishing Contact\nFirst:" << message.first_name <<
          "  Last:" << message.last_name << std::endl;

        this->address_book_publisher_->publish(message);
      };
```
- ```[this]```: the captured variable, which is the one in the same locality of this lambda expression which would be used later
- ```()```: it takes no parameters
- ```-> void```: it has no return type, it returns nothing
- ```{};```: the body of the lambda expression

Then create a new target for this node:
```
find_package(rclcpp REQUIRED)

# the target is publish_address_book, and it depends on another target called rclcpp
add_executable(publish_address_book src/publish_address_book.cpp)
ament_target_dependencies(publish_address_book rclcpp)

install(TARGETS
    publish_address_book
  DESTINATION lib/${PROJECT_NAME})
```

I've tried to use interfaces in the same package and failed. This following CMake code can fix that.
```
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(publish_address_book "${cpp_typesupport_target}")
```
>This finds the relevant generated C++ code from AddressBook.msg and allows your target to link against it.You may have noticed that this step was not necessary when the interfaces being used were from a different package that was built independently. This CMake code is only required when you want to use interfaces in the same package as the one in which they are defined.

Then, as always, build it, source it, and run it. The result:
```
docker@ros2:~/ws/src/ros2_ws$ ros2 run more_interfaces publish_address_book
Publishing Contact
First:John  Last:Doe
```
```
docker@ros2:~/ws/src/ros2_ws$ ros2 topic echo /address_book
first_name: John
last_name: Doe
phone_number: '1234567890'
phone_type: 2
---
```

>In this tutorial, you tried out different field types for defining interfaces, then built an interface in the same package where it’s being used. You also learned how to use another interface as a field type, as well as the package.xml, CMakeLists.txt, and #include statements necessary for utilizing that feature.

### Using parameters in a class (C++)
```
ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp
```
>Because you used the --dependencies option during package creation, you don’t have to manually add dependencies to package.xml or CMakeLists.txt.

Then examine the code:
```
class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("minimal_param_node")
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter is mine!";
    // Descriptors allow you to specify a text description of the parameter and its constraints, like making it read-only, specifying a range, etc.

    this->declare_parameter("my_parameter", "world", param_desc); // this declare the parameter named my_parameter valued string type "world"

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalParam::timer_callback, this)); 
      // this causes the timer_callback function to be executed every 1000ms
  }
  void timer_callback()
  {
    std::string my_param = this->get_parameter("my_parameter").as_string(); // gets the parameter and stores it in my_param

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    this->set_parameters(all_new_parameters); // this set the parameter my_parameter back to the default value world
    // this ensures it is always reset back to the original
  }
```

Then add this to the ```CMakeLists.txt``` file 
```
find_package(rclcpp REQUIRED)

add_executable(minimal_param_node src/cpp_parameters_node.cpp)
ament_target_dependencies(minimal_param_node rclcpp)

install(TARGETS
    minimal_param_node
  DESTINATION lib/${PROJECT_NAME}
)
```

Then, check dependencies, build it, source it, and run it.
```
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select cpp_parameters
source install/setup.bash
ros2 run cpp_parameters minimal_param_node
```
After running the node, the terminal shows the default value of the parameter. We can see the parameters
```
docker@ros2:~$ ros2 param list
/minimal_param_node:
  my_parameter
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  use_sim_time
```
To change it, use param set, enter this in another terminal:
```
ros2 param set /minimal_param_node my_parameter earth
```
The log info shown in the terminal:
```
[INFO] [1693707247.202344702] [minimal_param_node]: Hello world!
[INFO] [1693707248.202338617] [minimal_param_node]: Hello earth!
[INFO] [1693707249.202349402] [minimal_param_node]: Hello world!
```
However, the parameter would be set back to the default value every callback, so the set value "earth" only shown in one duration.

We can change the parameters via a launch file as well. In the package, create ```launch/cpp_parameters_launch.py```
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="minimal_param_node",
            name="custom_minimal_param_node",

            # we ensure our output is printed in our console.
            output="screen",
            emulate_tty=True,

            
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])
```
Then, add the launch file to ```CMakeLists.txt``` file 
```
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

Then, check it, build it, source it, then run it.
```
docker@ros2:~/ws/src/ros2_ws$ ros2 launch cpp_parameters cpp_parameters_launch.py
[INFO] [launch]: All log files can be found below /home/docker/.ros/log/2023-09-03-10-31-08-711418-ros2-568
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [minimal_param_node-1]: process started with pid [569]
[minimal_param_node-1] [INFO] [1693708270.685820721] [custom_minimal_param_node]: Hello earth!
[minimal_param_node-1] [INFO] [1693708271.685769066] [custom_minimal_param_node]: Hello world!
```

>You created a node with a custom parameter that can be set either from a launch file or the command line. You added the dependencies, executables, and a launch file to the package configuration files so that you could build and run them, and see the parameter in action.

### Using ```ros2doctor``` to identify issues
> When your ROS 2 setup is not running as expected, you can check its settings with the ```ros2doctor``` tool. ```ros2doctor``` checks all aspects of ROS 2, including platform, version, network, environment, running systems and more, and warns you about possible errors and reasons for issues.

```
ros2 doctor
```

Run turtle sim and teleop key, and run ros2doctor, it'll show that:
```
UserWarning: Publisher without subscriber detected on /turtle1/color_sensor.
UserWarning: Publisher without subscriber detected on /turtle1/pose.
```
> It seems that the /turtlesim node publishes data to two topics that aren’t being subscribed to, and ros2doctor thinks this could possibly lead to issues. If you run commands to echo the /color_sensor and /pose topics, those warnings will disappear because the publishers will have subscribers.

To get a full report:
```
ros2 doctor --report
```

**Crosscheck the information** from ```ros2 doctor``` and ```ros2 doctor --report``` to identify the issue. 
>For example, if ros2doctor returned the warning (mentioned earlier) that your distribution is “not fully supported or tested”, you might take a look at the ROS 2 INFORMATION section of the report:
```
distribution name      : <distro>
distribution type      : ros2
distribution status    : prerelease # it's not fully supported
release platforms      : {'<platform>': ['<version>']}
```

### Creating and using Plugins (C++)
>```pluginlib``` is a C++ library for **loading and unloading plugins from within a ROS package.** Plugins are dynamically loadable classes that are loaded from a runtime library (i.e. shared object, dynamically linked library). With pluginlib, one does not have to explicitly link their application against the library containing the classes – instead pluginlib can open a library containing exported classes at any point without the application having any prior awareness of the library or the header file containing the class definition. Plugins are useful for extending/modifying application behavior without needing the application source code.

Install the plugin library first:
```
sudo apt-get install ros-humble-pluginlib
```

>In this tutorial, you will create two new packages, one that defines the base class, and another that provides the plugins. The base class will define a generic polygon class, and then our plugins will define specific shapes.

First, create the package (it creates a node named area_node(where a source file ```area_node.cpp``` is created), we can tell it from the ```CMakeLists.txt``` file):
```
ros2 pkg create --build-type ament_cmake polygon_base --dependencies pluginlib --node-name area_node
```
Inside the include directory of the package, the /polygon_base/regular_polygon.hpp defines:
```
#ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
#define POLYGON_BASE_REGULAR_POLYGON_HPP

namespace polygon_base
{
  class RegularPolygon
  {
    public:
      virtual void initialize(double side_length) = 0;
      virtual double area() = 0;
      virtual ~RegularPolygon(){}

    protected:
      RegularPolygon(){} // this is needed with pluginlib
  };
}  // namespace polygon_base

#endif  // POLYGON_BASE_REGULAR_POLYGON_HPP
```
[This is the link about the syntax, virtual](https://shengyu7697.github.io/cpp-virtual/)
>**With pluginlib, a constructor without parameters is required, so if any parameters to the class are needed, we use the initialize method to pass them to the object.**

To make this header available to other classes, add to the ```CMakeLists.txt``` file:
```
install(
  DIRECTORY include/
  DESTINATION include
)
```
and add this before the ```ament_package``` command:
```
ament_export_include_directories(
  include
)
```

Then, create the second package, which is the plugin package. (it creates a package named ```polygon_plugins```, with dependencies ```polygon_base```(the one we just created) and ```pluginlib```) (it also creates a directory as the library named ```polygon_plugins```, under the ```include``` directory)(it also creates a source file ```polygon_plugins.cpp``` under ```src```)
```
ros2 pkg create --build-type ament_cmake polygon_plugins --dependencies polygon_base pluginlib --library-name polygon_plugins
```

In previous examples, we've use the code from another package, but in the form of static linking libraries. For now, with plugins, we can treate the other package as a dynamic linked library. We can tell that from the include syntax. One uses ```""```, and now we use ```<>```.

Inside the source file ```polygon_plugins.cpp```, it implements the abstract classes defined in package ```polygon_base```. But here's the special thing. The last three lines of the code:
```
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
```
(quoted from the tutorial)
The only piece that is pluginlib-specific is the last three lines, which invokes some magical macros that **register the classes as actual plugins**. Let’s go through the arguments to the PLUGINLIB_EXPORT_CLASS macro:
  1. The fully-qualified **type of the plugin class**, in this case, polygon_plugins::Square.
  2. The fully-qualified **type of the base class**, in this case, polygon_base::RegularPolygon

We've already make it so that the plugins can be created once the library is loaded, but the plugin loader still needs a way to find that library and to know what to reference within the library. Thus, we add another xml file, along with the two export lines above, makes everything available to the ROS toolchain.
```
<library path="polygon_plugins">
  <class type="polygon_plugins::Square" base_class_type="polygon_base::RegularPolygon">
    <description>This is a square plugin.</description>
  </class>
  <class type="polygon_plugins::Triangle" base_class_type="polygon_base::RegularPolygon">
    <description>This is a triangle plugin.</description>
  </class>
</library>
# this specifies the library the plugin loader wants to find.
# the references within this library are specified as base_class_type
```
(quoted from the tutorial)
1. The library tag gives the relative path to a library that contains the plugins that we want to export. In ROS 2, that is just the name of the library, which is ```polygon_plugins```

2. **The class tag declares a plugin that we want to export from our library.** Let’s go through its parameters:
    - ```type```: The fully qualified type of the plugin. For us, that’s polygon_plugins::Square.
    - ```base_class```: The fully qualified base class type for the plugin. For us, that’s polygon_base::RegularPolygon.
    - ```description```: A description of the plugin and what it does.

After specifying the xml file, the last step is to export the plugins via ```CMakeLists.txt```. This is a change from ROS 1, where the exporting was done via package.xml. Add the following line to your ros2_ws/src/polygon_plugins/CMakeLists.txt after the line reading find_package(pluginlib REQUIRED):
```
pluginlib_export_plugin_description_file(polygon_base plugins.xml)
```
1. (the package with the base class ```polygon_base```)
2. (export the plugin as the ```plugins.xml``` specified)

Now, we can use this package as plugins in any other packages. For this tutorial, we use the plugins in the base package ```polygon_base```.
```
#include <pluginlib/class_loader.hpp>
#include <polygon_base/regular_polygon.hpp>

int main(int argc, char** argv)
{
  // To avoid unused parameter warnings
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("polygon_base", "polygon_base::RegularPolygon");

  try
  {
    std::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createSharedInstance("polygon_plugins::Triangle");
    triangle->initialize(10.0);

    std::shared_ptr<polygon_base::RegularPolygon> square = poly_loader.createSharedInstance("polygon_plugins::Square");
    square->initialize(10.0);

    printf("Triangle area: %.2f\n", triangle->area());
    printf("Square area: %.2f\n", square->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}
```
>There are a number of ways to instantiate an instance of the class. In this example, we’re using shared pointers. We just need to call ```createSharedInstance``` with the fully-qualified type of the plugin class, in this case, ```polygon_plugins::Square```.

>Important note: the ```polygon_base``` package in which this node is defined does NOT depend on the ```polygon_plugins``` class. **The plugins will be loaded dynamically without any dependency needing to be declared**. Furthermore, we’re instantiating the classes with hardcoded plugin names, but you can also do so dynamically with parameters, etc.

Then, build it, source it, then run it:
```
colcon build --packages-select polygon_base polygon_plugins
source install/setup.bash
ros2 run polygon_base area_node
```
```
Triangle area: 43.30
Square area: 100.00
```

## Intermediate

### Managing Dependecies with ```rosdep```

>rosdep is a dependency management utility that can work with packages and external libraries. It is a command-line utility for identifying and installing dependencies to build or install a package. rosdep is not a package manager in its own right; it is a meta-package manager that uses its own knowledge of the system and the dependencies to find the appropriate package to install on a particular platform. The actual installation is done using the system package manager (e.g. apt on Debian/Ubuntu, dnf on Fedora/RHEL, etc). It is most often invoked before building a workspace, where it is used to install the dependencies of the packages within that workspace. It has the ability to work over a single package or over a directory of packages (e.g. workspace).

The package.xml is the file in your software where rosdep finds the set of dependencies. It is important that the list of dependencies in the package.xml is complete and correct, which allows all of the tooling to determine the packages dependencies. The dependencies in the package.xml file are generally referred to as “rosdep keys”.
- **```<depend>```**: These are dependencies that should be provided at both build time and run time for your package. For C++ packages, if in doubt, use this tag. Pure Python packages generally don’t have a build phase, so should never use this and should use <exec_depend> instead.
- **```<build_depend>```**: If you only use a particular dependency for building your package, and not at execution time, you can use the <build_depend> tag. With this type of dependency, an installed binary of your package does not require that particular package to be installed.
- **```<build_export_depend>```**: This tag is used when your package exports headers or other resources that depend on a specific package. It ensures that packages depending on your package for building have access to these resources.
- **```<exec_depend>```**: This tag declares dependencies for shared libraries, executables, Python modules, launch scripts and other files required when running your package.
- **```<test_depend>```**:his tag declares dependencies needed only by tests. Dependencies here should not be duplicated with keys specified by <build_depend>, <exec_depend>, or <depend>

>rosdep will check for package.xml files in its path or for a specific package and find the rosdep keys stored within. These keys are then cross-referenced against a central index to find the appropriate ROS package or software library in various package managers. Finally, once the packages are found, they are installed and ready to go! rosdep works by retrieving the central index on to your local machine so that it doesn’t have to access the network every time it runs (on Debian/Ubuntu the configuration for it is stored in /etc/ros/rosdep/sources.list.d/20-default.list). The central index is known as rosdistro, which may be found online. We’ll explore that more in the next section.

Install all the dependencies over a workspace with root src.
```
rosdep install --from-paths src -y --ignore-src
```
- --from-paths src specifies the path to check for package.xml files to resolve keys for
- -y means to default yes to all prompts from the package manager to install without prompts
- --ignore-src means to ignore installing dependencies, even if a rosdep key exists, if the package itself is also in the workspace.

There are additional arguments and options available. Use rosdep -h to see them.

### Creating an action
Actions are defined in .action files of the form:
```
# Request
---
# Result
---
# Feedback
```
- request: initiating a new goal
- result: goal is done
- feedback: periodical updates about the goal
- goal: the instance of an action

Create ```/action/Fibonacci.action``` inside the package:
```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```
Before we can use the action interface, remember we should transfer it into C-code. Thus, we should use the interface generator included in ```CMakeLists.txt```.
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```
Also, we should change the ```package.xml``` for that build tool and cpp header source.
```
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>action_msgs</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Then, build it, source it, then run it
```
colcon build
source install/setup.bash
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```
And here's the result:
```
docker@ros2:~/ws/src/ros2_ws$ ros2 interface show action_tutorials_interfaces/action/Fibonacci
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

### Writing an action server and client (C++)

First, create a package
```
# in /ro2_ws/src
ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp
```
> In order to make the package compile and work on Windows, we need to add in some “visibility control”. 
Create ```action_tutorials_cpp/include/action_tutorials_cpp/visibility_control.h``` and put:
```
#ifndef ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
#define ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((dllexport))
    #define ACTION_TUTORIALS_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define ACTION_TUTORIALS_CPP_EXPORT __declspec(dllexport)
    #define ACTION_TUTORIALS_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACTION_TUTORIALS_CPP_BUILDING_DLL
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_EXPORT
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_IMPORT
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE ACTION_TUTORIALS_CPP_PUBLIC
  #define ACTION_TUTORIALS_CPP_LOCAL
#else
  #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((visibility("default")))
  #define ACTION_TUTORIALS_CPP_IMPORT
  #if __GNUC__ >= 4
    #define ACTION_TUTORIALS_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define ACTION_TUTORIALS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC
    #define ACTION_TUTORIALS_CPP_LOCAL
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
```
Honestly, I don't understand what is going on here. But we can start writing an action server computing Fibonacci. Create ```/src/fibonacci_action_server.cpp```. We should also add a few lines to ```CMakeLists.txt``` to maintain dependencies.
[About cpp syntax explicit](https://shengyu7697.github.io/cpp-explicit/)

```
# ... find packages

add_library(action_server SHARED
  src/fibonacci_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```
Then run ```colcon build```. It should build ``` fibonacci_action_server.cpp``` in ```action_tutorials_cpp``` pacakge.

We also instantiate a ROS timer that will kick off the one and only call to send_goal:
```
this->timer_ = this->create_wall_timer(
  std::chrono::milliseconds(500),
  std::bind(&FibonacciActionClient::send_goal, this));
```
[About std::bind](https://www.jyt0532.com/2017/01/08/bind/)

And similarly, build the client as well. ```CMakeLists.txt```:
```
add_library(action_client SHARED
  src/fibonacci_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```
```colcon build``` should compile the entire workspace, including the ```fibonacci_action_client.cpp``` in the ```action_tutorials_cpp``` package.

After that, source it then run it. Here's the result:
- client
```
docker@ros2:~/ws/src/ros2_ws$ ros2 run action_tutorials_cpp fibonacci_action_client
[INFO] [1693889845.150850325] [fibonacci_action_client]: Sending goal
[INFO] [1693889845.152368070] [fibonacci_action_client]: Goal accepted by server, waiting for result
[INFO] [1693889845.153103892] [fibonacci_action_client]: Next number in sequence received: 0 1 1
[INFO] [1693889846.152979478] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 
[INFO] [1693889847.152965399] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 3 
[INFO] [1693889848.152860585] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 3 5 
[INFO] [1693889849.153781401] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 
[INFO] [1693889850.153472725] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 
[INFO] [1693889851.152991279] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 21 
[INFO] [1693889852.153055651] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 21 34 
[INFO] [1693889853.153347930] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 21 34 55 
[INFO] [1693889854.153263697] [fibonacci_action_client]: Result received: 0 1 1 2 3 5 8 13 21 34 55 
```
- server(it keeps running after the first goal is succeeded and waits for another request)
```
docker@ros2:~/ws/src/ros2_ws$ ros2 run action_tutorials_cpp fibonacci_action_server
[INFO] [1693889845.151792653] [fibonacci_action_server]: Received goal request with order 10
[INFO] [1693889845.152439773] [fibonacci_action_server]: Executing goal
[INFO] [1693889845.152817284] [fibonacci_action_server]: Publish feedback
[INFO] [1693889846.152782472] [fibonacci_action_server]: Publish feedback
[INFO] [1693889847.152374881] [fibonacci_action_server]: Publish feedback
[INFO] [1693889848.152729281] [fibonacci_action_server]: Publish feedback
[INFO] [1693889849.153213684] [fibonacci_action_server]: Publish feedback
[INFO] [1693889850.153044611] [fibonacci_action_server]: Publish feedback
[INFO] [1693889851.152767172] [fibonacci_action_server]: Publish feedback
[INFO] [1693889852.152774342] [fibonacci_action_server]: Publish feedback
[INFO] [1693889853.153034720] [fibonacci_action_server]: Publish feedback
[INFO] [1693889854.152972188] [fibonacci_action_server]: Goal succeeded

```

### Composing multiple nodes in a single process
[About components](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html)
This tutorial uses rclcpp_components, ros2component, and composition packages. Before we start, we should download the composition package into our workspace. That package ```composition``` is a demo package that runs in overlay. The other two packages, which are ```rclcpp_components``` and ```ros2component```, are packages that runs in underlay and already downloaded in the ros distro ```/opt/ros/humble```. (It took me a while to realize that :D)

- [About ament_cmake](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html)
    > ament_cmake is the build system for CMake based packages in ROS 2 (in particular, it will be used for most C/C++ projects). 

So, we first download the package ```composition``` into our workspace, then build it, and source it.
```
colcon build # just build the whole workspace
source install/setup.bash
```

To see what components are registered and available in the workspace:
```
docker@ros2:~/ws/src/ros2_ws$ ros2 component types
composition
  composition::Talker
  composition::Listener
  composition::NodeLikeListener
  composition::Server
  composition::Client
action_tutorials_cpp
  action_tutorials_cpp::FibonacciActionServer
  action_tutorials_cpp::FibonacciActionClient
robot_state_publisher
  robot_state_publisher::RobotStatePublisher
tf2_ros
  tf2_ros::StaticTransformBroadcasterNode
```
These are the available components, which is the ROS2 APIs.

#### Run-time composition using ROS services with a publisher and subscriber
Run the node ```component_containter``` in package ```rclcpp_components``` first so that we can load components to run.
```
ros2 run rclcpp_components component_container
```
Then, in the second shell, verify that the component container is running properly:
```
docker@ros2:~/ws/src/ros2_ws$ ros2 component list
/ComponentManager
```
Once the container is running properly, we can now load the component ```composition::Talker``` inside the package ```composition``` into the container ```/ComponentManager```.
```
docker@ros2:~/ws/src/ros2_ws$ ros2 component load /ComponentManager composition composition::Talker
Loaded component 1 into '/ComponentManager' container node as '/talker'
```
Then, load the listener into the container as well.
```
docker@ros2:~/ws/src/ros2_ws$ ros2 component load /ComponentManager composition composition::Listener
Loaded component 2 into '/ComponentManager' container node as '/listener'
```
Now, we check the state of the components here:
```
docker@ros2:~/ws/src/ros2_ws$ ros2 component list
/ComponentManager
  1  /talker
  2  /listener
```
Once the listner and the talker are running, in the first terminal where the container is executed, messages are shown:
```
...
[INFO] [1693994957.099389176] [talker]: Publishing: 'Hello World: 108'
[INFO] [1693994957.099621378] [listener]: I heard: [Hello World: 108]
...
```
We can unload the components using the unique id:
```
ros2 component unload /ComponentManager 1 2
```
```
Unloaded component 1 from '/ComponentManager' container
Unloaded component 2 from '/ComponentManager' container
```
#### Run-time composition using ROS services with a server and client
Similar to the first example, we first execute the container in the first terminal, then run the server and client in the second terminal.
```
ros2 run rclcpp_components component_container
```
```
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client
```

#### Compile-time composition using ROS services
>This demos shows that **the same shared libraries can be reused to compile a single executable running multiple components**. The executable contains all four components from above: talker and listener as well as server and client.

To run all the components with the package's own source file:
```
ros2 run composition manual_composition
```

#### Run-time composition using dlopen
> This demo presents an alternative to run-time composition by creating a generic container process and explicitly passing the libraries to load without using ROS interfaces. The process will open each library and create one instance of each “rclcpp::Node” class in the library source code.

```
docker@ros2:~/ws/src/ros2_ws$ ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so
[INFO] [1693996242.201452547] [dlopen_composition]: Load library /home/docker/ws/src/ros2_ws/src/install/composition/lib/libtalker_component.so
[INFO] [1693996242.274358002] [dlopen_composition]: Instantiate class rclcpp_components::NodeFactoryTemplate<composition::Talker>
[INFO] [1693996242.660578628] [dlopen_composition]: Load library /home/docker/ws/src/ros2_ws/src/install/composition/lib/liblistener_component.so
[INFO] [1693996242.774907073] [dlopen_composition]: Instantiate class rclcpp_components::NodeFactoryTemplate<composition::Listener>
[INFO] [1693996243.660757866] [talker]: Publishing: 'Hello World: 1'
[INFO] [1693996243.671404989] [listener]: I heard: [Hello World: 1]
```
[About .so file](https://superuser.com/questions/71404/what-is-an-so-file)

#### Composition using launch actions
While the command line tools are useful for debugging and diagnosing component configurations, it is frequently more convenient to start a set of components at the same time. 
```
ros2 launch composition composition_demo_launch.py
```
(Note that there's a typo on the tutorial: ```composition_demo.launch.py``` -> ```composition_demo_launch.py```)
And here's the result:
```
[INFO] [launch]: All log files can be found below /home/docker/.ros/log/2023-09-06-18-33-58-732464-ros2-9919
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [component_container-1]: process started with pid [9932]
[component_container-1] [INFO] [1693996446.892453985] [my_container]: Load Library: /home/docker/ws/src/ros2_ws/src/install/composition/lib/libtalker_component.so
[component_container-1] [INFO] [1693996446.975510948] [my_container]: Found class: rclcpp_components::NodeFactoryTemplate<composition::Talker>
[component_container-1] [INFO] [1693996446.975592848] [my_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<composition::Talker>
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/talker' in container '/my_container'
[component_container-1] [INFO] [1693996447.333908942] [my_container]: Load Library: /home/docker/ws/src/ros2_ws/src/install/composition/lib/liblistener_component.so
[component_container-1] [INFO] [1693996447.463670364] [my_container]: Found class: rclcpp_components::NodeFactoryTemplate<composition::Listener>
[component_container-1] [INFO] [1693996447.463759565] [my_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<composition::Listener>
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/listener' in container '/my_container'
[component_container-1] [INFO] [1693996448.098694799] [talker]: Publishing: 'Hello World: 1'
[component_container-1] [INFO] [1693996448.115580893] [listener]: I heard: [Hello World: 1]
[component_container-1] [INFO] [1693996449.098506663] [talker]: Publishing: 'Hello World: 2'
[component_container-1] [INFO] [1693996449.099018766] [listener]: I heard: [Hello World: 2]
```

#### Some other usage
We can remap the container name and namespace. For this example, the original node name was ComponentManager. Now, we name it as ```Mycontainer```, with namespace named ```ns```:
```
ros2 run rclcpp_components component_container --ros-args -r __node:=MyContainer -r __ns:=/ns
```
So, we load components in such way:
```
ros2 component load /ns/MyContainer composition composition::Listener
```

Components names and namespaces can also be remapped:
```
ros2 component load /ComponentManager composition composition::Talker --node-name talker2
ros2 component load /ComponentManager composition composition::Talker --node-namespace /ns
ros2 component load /ComponentManager composition composition::Talker --node-name talker3 --node-namespace /ns2
```
Here's the result of ```ros2 component list```
```
/ComponentManager
   1  /talker2
   2  /ns/talker
   3  /ns2/talker3
```

### Monitoring for parameter changes (C++)
Learn how to use ```ParameterEventHandler``` class to monitor the parameter changes to itself or others. However, this tutorial must be running the Galactic distribution of ROS2, while the one run by docker is the humble distribution. So, I'll skip this part for now.

### Launch
ROS2 Launch files can run and configure multiple executables simultaneously, running multiple ROS2 nodes.
#### Creating a Launch File
The launch files can be written in python, xml, or yaml files. [This is the article about xml, json, and yaml](https://www.freecodecamp.org/news/what-is-yaml-the-yml-file-format/#:~:text=YAML%20is%20a%20human%2Dreadable,application%20using%20a%20standard%20format.)
They are all configuration files for applications. I decided to try all three types of launch files. First, under the workspace direcotry ```ros2_ws```, create a directory ```launch``` to store all the launch files. Then, create the three launch files:
```
mkdir launch
cd launch
touch turtlesim_mimic_launch.py\
  turtlesim_mimic_launch.xml\
  turtlesim_mimic_launch.yaml
```
- For python:
  ```
  from launch import LaunchDescription
  from launch_ros.actions import Node

  def generate_launch_description():
      return LaunchDescription([
          Node(
              package='turtlesim',
              namespace='turtlesim1',
              executable='turtlesim_node',
              name='sim'
          ),
          Node(
              package='turtlesim',
              namespace='turtlesim2',
              executable='turtlesim_node',
              name='sim'
          ),
          Node(
              package='turtlesim',
              executable='mimic',
              name='mimic',
              remappings=[
                  ('/input/pose', '/turtlesim1/turtle1/pose'),
                  ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
              ]
          )
      ])
  ```
- For xml
  ```
  <launch>
    <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim1"/>
    <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim2"/>
    <node pkg="turtlesim" exec="mimic" name="mimic">
      <remap from="/input/pose" to="/turtlesim1/turtle1/pose"/>
      <remap from="/output/cmd_vel" to="/turtlesim2/turtle1/cmd_vel"/>
    </node>
  </launch>
  ```
- For yaml
  ```
  launch:

  - node:
      pkg: "turtlesim"
      exec: "turtlesim_node"
      name: "sim"
      namespace: "turtlesim1"

  - node:
      pkg: "turtlesim"
      exec: "turtlesim_node"
      name: "sim"
      namespace: "turtlesim2"

  - node:
      pkg: "turtlesim"
      exec: "mimic"
      name: "mimic"
      remap:
      -
          from: "/input/pose"
          to: "/turtlesim1/turtle1/pose"
      -
          from: "/output/cmd_vel"
          to: "/turtlesim2/turtle1/cmd_vel"
  ```

Then, run the launch files:
```
ros2 launch turtlesim_mimic_launch.<file_type>

# file type could be .py .xml .yaml
```
Two turtlesim windows would show up. Also, note that it's possible that the launch file is inside a package. So that we should specify the package name while we're launching a file:
```
ros2 launch <package_name> <launch_file_name>
```
**Also, for packages that has launch file, it's important to add**
```
<exec_depend>ros2launch</exec_depend>
```
**to the ```package.xml``` file. This helps make sure that the ros2 launch command is available after building your package. It also ensures that all launch file formats are recognized. (quoted from the tutorial)**

Here's what happened after launching the files:
```
docker@ros2:~/ws/src/ros2_ws/launch$ ros2 launch turtlesim_mimic_launch.py
[INFO] [launch]: All log files can be found below /home/docker/.ros/log/2023-09-07-13-09-46-732492-ros2-153
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [turtlesim_node-1]: process started with pid [154]
[INFO] [turtlesim_node-2]: process started with pid [156]
[INFO] [mimic-3]: process started with pid [158]
[turtlesim_node-2] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-docker'
[turtlesim_node-1] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-docker'
[turtlesim_node-1] [INFO] [1694063387.082140704] [turtlesim1.sim]: Starting turtlesim with node name /turtlesim1/sim
[turtlesim_node-2] [INFO] [1694063387.082376907] [turtlesim2.sim]: Starting turtlesim with node name /turtlesim2/sim
[turtlesim_node-2] [INFO] [1694063387.089878707] [turtlesim2.sim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
[turtlesim_node-1] [INFO] [1694063387.089934608] [turtlesim1.sim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
```
Now, in another terminal, we use ```ros2 pub``` to publish informations onto the topic so that the turtle could start moving:
```
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-09-07%20131638.png)

Now, in another terminal, run ```rqt_graph``` to see the whole graph of current running nodes. Remember to source the setup file first:
```
source install/setup.bash
rqt_graph
```
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-09-07%20132221.png)

#### Integrating launch files into ROS2 packages
First, build another workspace ```launch_ws``` and create an ```src``` directory underneath it. Afterwards, create a pacakge for demonstration:
```
mkdir -p launch_ws/src
cd launch_ws/src
ros2 pkg create cpp_launch_example --build-type ament_cmake
```
Now, add a few lines to ```CMakeLists.txt``` to install the launch files:
```
# at the end of the file
# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# before ament_package()
ament_package()
```
Then, create a ```launch``` directory  and create three launch files:
```
mkdir launch
cd launch
touch my_script_launch.py my_script_launch.xml my_script_launch.yaml
```
- For python
  > **Inside your launch directory, create a new launch file called my_script_launch.py. _launch.py is recommended, but not required, as the file suffix for Python launch files. However, the launch file name needs to end with launch.py to be recognized and autocompleted by ros2 launch.**
  ```
  import launch
  import launch_ros.actions

  def generate_launch_description(): # this function must be named like this, and the return type is fixed as well
    return launch.LaunchDescription([
      launch_ros.actions.Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='talker'),
    ])
  ```
- For xml
  > **Inside your launch directory, create a new launch file called my_script_launch.xml. _launch.xml is recommended, but not required, as the file suffix for XML launch files.**
  ```
  <launch>
    <node pkg="demo_nodes_cpp" exec="talker" name="talker"/>
  </launch>
  ```
- For yaml
  > **Inside your launch directory, create a new launch file called my_script_launch.yaml. _launch.yaml is recommended, but not required, as the file suffix for YAML launch files.**
  ```
  launch:

  - node:
    pkg: "demo_nodes_cpp"
    exec: "talker"
    name: "talker"
  ```
Then, build it(in the root directory of the workspace), source it, then launch it.
```
colcon build
source install/setup.bash
ros2 launch cpp_launch_example my_script_launch.<file_type>
# file_type could be .py .xml .yaml
```
By the way, remember to install the package ```demo_nodes_cpp``` beforehand. Run the following code:
```
sudo apt-get install ros-humble-demo-nodes-cpp
source install/setup.bash
```
This would install the package, note that ```humble``` term is the ros2 distro. If we're using other distros, then substitute the token with other distro names. After the installation, you should see the package inside the diectory ```/opt/ros/humble/share/demo_nodes_cpp```.

**Remember that, if there's any packages you want to install in the future about ros2, use the command
```
sudo apt-get install ros-<distro>-<pacakge_name>
```
Note that the package name should all be connected with "-" instead of "_". Then, the package with name connected with "\_" would be installed afterwards.

#### Using substitutions
Substitutions: 
1. variables that are only evaluated during execution of the launch file
2. can be used to acquire specific information like a launch configuration, an environment variable, or to evaluate an arbitrary Python expression.

> This tutorial shows usage examples of substitutions in ROS 2 launch files.

First, create a pacakge under the ```launch_ws``` workspace, and make a directory ```launch``` in it.
```
ros2 pkg create launch_tutorial --build-type ament_python
mkdir launch_tutorial/launch
```

Then, change the setup.py so that the launch files will be installed.
```
import os # new
from glob import glob
from setuptools import setup

package_name = 'launch_tutorial'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ]
)
```
This part of the code is hardly understandable by first glance. So I went through a few links:
- [About os.path.join()](https://www.geeksforgeeks.org/python-os-path-join-method/)
- [About glob.glob()](https://ithelp.ithome.com.tw/articles/10262521)
- Also, the almighty ChatGPT
First, the setup function defines the package's metadata and configuration. ```data_files``` is a list of 2-tuples where each tuple represents a destination directory and a list of files to include, which means include that list of files into the destination directory.

Then, let's focus on the tuple we added. The first element of the tuple is the destination, which the ```os.path.join()``` method would return a string ```share/launch_tutorial/launch```, where we want to include the file described by the returned list of file names by the ```glob``` function.

The ```glob``` function would return a list of file paths after searching ```/launch/*launch.[pxy][yma]*``` returned by ```os.path.join()```. What is that absurd string? Well, it's special syntax for ```glob``` function. Just like how it behaves in Unix like system, * means everything. And ```[pxy][yma]``` would search for all with tokens ```py```, ```xm```, and ```ya```. Is it clear now? It'll search for launch files in the ```.py```, ```.xml```, and ```.yaml``` formats. 

Next, in the ```launch``` directory, create a **launch file that will call and pass arguments to another launch file**. ```example_main.launch.py``` (remember that the python launch files should be suffixed "launch.py")
```
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'launch',
                    'example_substitutions.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
```

And create ```example_substitutions.launch.py``` in the ```launch``` directory as well.
```
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle,
        change_background_r,
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])
```

Then, as always, build it, source it.
```
colcon build
source install/setup.bash
```
However, I ran into a some issues while building it. Here's the error message:
```
/usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated.
```
I googled it [here](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/) and turns out that the setuptools package was too new. The original version I had was 59.6.0, but the last available version was 58.2.0. So that I ```pip3 install setuptools==58.2.0```, and source the setup files again. After that, the build was done successfully.

Finally, we can launch it:
```
ros2 launch launch_tutorial example_main.launch.py
```
(quoted from the tutorial) Then the following four things would happen:
1. Start a turtlesim node with a blue background
2. Spawn the second turtle
3. Change the color to purple
4. Change the color to pink after two seconds if the provided ```background_r``` argument is 200 and ```use_provided_red``` argument is ```True```

We can also modify launch arguments that isn't neccessarily defined in the launch file and could be done in the command line. The following command would show the arguments
```
ros2 launch launch_tutorial example_substitutions.launch.py --show-args
```
And it would return the argument configurations:
```
Arguments (pass arguments as '<name>:=<value>'):

    'turtlesim_ns':
        no description given
        (default: 'turtlesim1')

    'use_provided_red':
        no description given
        (default: 'False')

    'new_background_r':
        no description given
        (default: '200')
```
Which correspond with the code in ```example_substitutions.launch.py```:
```
turtlesim_ns_launch_arg = DeclareLaunchArgument(
    'turtlesim_ns',
    default_value='turtlesim1'
)
use_provided_red_launch_arg = DeclareLaunchArgument(
    'use_provided_red',
    default_value='False'
)
new_background_r_launch_arg = DeclareLaunchArgument(
    'new_background_r',
    default_value='200'
)
```
And we can modify those argument setups by:
```
ros2 launch launch_tutorial example_substitutions.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```

#### Using Event Handler
> **Launch in ROS 2 is a system that executes and manages user-defined processes. It is responsible for monitoring the state of processes it launched, as well as reporting and reacting to changes in the state of those processes. These changes are called events and can be handled by registering an event handler with the launch system. Event handlers can be registered for specific events and can be useful for monitoring the state of processes. Additionally, they can be used to define a complex set of rules which can be used to dynamically modify the launch file.**

In the package ```launch_tutorial```, create a file ```example_event_handlers.launch.py``` in ```launch```.
```
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=turtlesim_node,
                on_start=[
                    LogInfo(msg='Turtlesim started, spawning turtle'),
                    spawn_turtle
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawn_turtle,
                on_stdout=lambda event: LogInfo(
                    msg='Spawn request says "{}"'.format(
                        event.text.decode().strip())
                )
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_turtle,
                on_completion=[
                    LogInfo(msg='Spawn finished'),
                    change_background_r,
                    TimerAction(
                        period=2.0,
                        actions=[change_background_r_conditioned],
                    )
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=turtlesim_node,
                on_exit=[
                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                            ' closed the turtlesim window')),
                    EmitEvent(event=Shutdown(
                        reason='Window closed'))
                ]
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])
```

The ```RegisterEventHandler()``` would register event handlers, which are a few actions, such as OnProcessStart, OnProcessIO, OnExecutionComplete, OnProcessExit, and OnShutdown. The events were defined in the description.

After that, build it, source it, then run it.
```
colcon build
source install/setup.bash
ros2 launch launch_tutorial example_event_handlers.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```
(remember that these are argument setups)

![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-09-08%20082025.png)
The log infos:
```
[INFO] [launch]: All log files can be found below /home/docker/.ros/log/2023-09-08-08-19-47-605832-ros2-210
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [turtlesim_node-1]: process started with pid [211]
[INFO] [launch.user]: Turtlesim started, spawning turtle #
[INFO] [Spawn "{x: 2, y: 2, theta: 0.2}"-2]: process started with pid [213]
[turtlesim_node-1] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-docker'
[turtlesim_node-1] [INFO] [1694132390.797659651] [turtlesim3.sim]: Starting turtlesim with node name /turtlesim3/sim
[turtlesim_node-1] [INFO] [1694132390.922368520] [turtlesim3.sim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
[INFO] [launch.user]: Spawn request says "requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.2, name='')" #
[turtlesim_node-1] [INFO] [1694132394.946573205] [turtlesim3.sim]: Spawning turtle [turtle2] at x=[2.000000], y=[2.000000], theta=[0.200000]
[INFO] [launch.user]: Spawn request says "response: #
turtlesim.srv.Spawn_Response(name='turtle2')"
[INFO] [Spawn "{x: 2, y: 2, theta: 0.2}"-2]: process has finished cleanly [pid 213]
[INFO] [launch.user]: Spawn finished #
[INFO] [sim background_r 120-3]: process started with pid [250]
[INFO] [sim background_r 200-4]: process started with pid [253]
[INFO] [sim background_r 120-3]: process has finished cleanly [pid 250]
[INFO] [sim background_r 200-4]: process has finished cleanly [pid 253]
[INFO] [turtlesim_node-1]: process has finished cleanly [pid 211]
[INFO] [launch.user]: docker closed the turtlesim window #
[INFO] [launch.user]: Launch was asked to shutdown: Window closed #
```
(quoted from the tutorial)

This will do the following:
1. Start a turtlesim node with a blue background
2. Spawn the second turtle
3. Change the color to purple
4. Change the color to pink after two seconds if the provided background_r argument is 200 and use_provided_red argument is True
5. Shutdown the launch file when the turtlesim window is closed

Additionally, it will log messages to the console when:
1. The turtlesim node starts
2. The spawn action is executed
3. The change_background_r action is executed
4. The change_background_r_conditioned action is executed
5. The turtlesim node exits
6. The launch process is asked to shutdown.

#### Managing large projects
> This tutorial describes some tips for writing launch files for large projects. **The focus is on how to structure launch files so they may be reused as much as possible in different situations**. Additionally, it covers usage examples of different ROS 2 launch tools, like parameters, YAML files, remappings, namespaces, default arguments, and RViz configs.

This tutorial is using turtle_tf2_py package, so we should install that before getting started
```
sudo apt-get install ros-humble-turtle-tf2-py
```

##### Top Level organization
Running ROS2 in a robot may involve many nodes which are responsible for many different tasks. So we're simulating that circumstance with multiple nodes related to turtlesim.

First, we go through the **top level** launch file structure, create ```launch_turtlesim.launch.py``` under ```launch``` directory:
```
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   turtlesim_world_1 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_world_1.launch.py'])
      )
   turtlesim_world_2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_world_2.launch.py'])
      )
   broadcaster_listener_nodes = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/broadcaster_listener.launch.py']),
      launch_arguments={'target_frame': 'carrot1'}.items(),
      )
   mimic_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/mimic.launch.py'])
      )
   fixed_frame_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/fixed_broadcaster.launch.py'])
      )
   rviz_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_rviz.launch.py'])
      )

   return LaunchDescription([
      turtlesim_world_1,
      turtlesim_world_2,
      broadcaster_listener_nodes,
      mimic_node,
      fixed_frame_node,
      rviz_node
   ])
```
(quoted from the tutorial)
- This launch file includes a set of other launch files. Each of these included launch files contains nodes, parameters, and possibly, nested includes, which pertain to one part of the system. To be exact, we launch two turtlesim simulation worlds, TF broadcaster, TF listener, mimic, fixed frame broadcaster, and RViz nodes.
- The top-level launch file should be short. Details are included by the low-level launch files. However, we should be aware of the tradeoffs.

##### Parameters
Now, we should set parameters in the launch file structure, so create ```turtlesim_world_1.launch.py```:
```
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
   background_r_launch_arg = DeclareLaunchArgument(
      'background_r', default_value=TextSubstitution(text='0')
   )
   background_g_launch_arg = DeclareLaunchArgument(
      'background_g', default_value=TextSubstitution(text='84')
   )
   background_b_launch_arg = DeclareLaunchArgument(
      'background_b', default_value=TextSubstitution(text='122')
   )

   return LaunchDescription([
      background_r_launch_arg,
      background_g_launch_arg,
      background_b_launch_arg,
      Node(
         package='turtlesim',
         executable='turtlesim_node',
         name='sim',
         parameters=[{
            'background_r': LaunchConfiguration('background_r'),
            'background_g': LaunchConfiguration('background_g'),
            'background_b': LaunchConfiguration('background_b'),
         }]
      ),
   ])
```
The code above launches a turtlesim node simulation, with configuration parameters defined and passed to it.

Now, we want to create another turtlesim simulation, so create ```turtlesim_world_2.launch.py```:
```
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('launch_tutorial'),
      'config',
      'turtlesim.yaml'
      )

   return LaunchDescription([
      Node(
         package='turtlesim',
         executable='turtlesim_node',
         namespace='turtlesim2',
         name='sim',
         parameters=[config]
      )
   ])
```
This would load the configuration parameters directly from the ```turtlesim.yaml``` file. With ```yaml``` files, it's easier to manage large configuration set of parameters. In addition, ```yaml``` files can be easily exported from the current ros2 param list. So now we create a directory ```config``` in our package, then create ```turtlesim.yaml``` inside.
```
/turtlesim2/sim:
   ros__parameters:
      background_b: 255
      background_g: 86
      background_r: 150
```

However, that set of configuration parameters are only for the node ```sim``` in the ```turtlesim2``` namespace. What if we want to set these config parameters to multiple nodes in different namespaces? We can use the **wildcard** syntax -> ```/**```. In this way, all the launch files include this ```yaml``` file could apply this set of parameters. Thus, we could change ```turtlesim.yaml``` to:
```
/**:
   ros__parameters:
      background_b: 255
      background_g: 86
      background_r: 150
```
Also, create ```turtlesim_world_3.launch.py``` which could demonstrate this syntax:
```
# same as turtlesim_world_2.launch.py but just add another node
Node(
   package='turtlesim',
   executable='turtlesim_node',
   namespace='turtlesim3',
   name='sim',
   parameters=[config]
)
```

##### Namespaces
(quoted from the tutorial)
Using unique namespaces allow the system to start two similar nodes without node name or topic name conflicts. However, if the launch file contains a large number of nodes, defining namespaces for each of them can become tedious. To solve that issue, **the ```PushRosNamespace``` action can be used to define the global namespace for each launch file description. Every nested node will inherit that namespace automatically.**

First, remove the namespace specification ```namespace='turtlesim2'``` from the launch file first. Then, in the top-level launch file ```launch_turtlesim.launch.py```, add:
```
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

   ...
   turtlesim_world_2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_world_2.launch.py'])
      )
   turtlesim_world_2_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('turtlesim2'),
         turtlesim_world_2,
      ]
   )
```
Replace ```turtlesim_world_2``` with ```turtlesim_world_2_with_namespace``` which has PushRosNamespace in the first place.

##### Reusing the nodes
Create a launch file ```broadcaster_listener.launch.py```:
```
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      DeclareLaunchArgument(
         'target_frame', default_value='turtle1',
         description='Target frame name.'
      ),
      Node(
         package='turtle_tf2_py',
         executable='turtle_tf2_broadcaster',
         name='broadcaster1',
         parameters=[
            {'turtlename': 'turtle1'}
         ]
      ),
      Node(
         package='turtle_tf2_py',
         executable='turtle_tf2_broadcaster',
         name='broadcaster2',
         parameters=[
            {'turtlename': 'turtle2'}
         ]
      ),
      Node(
         package='turtle_tf2_py',
         executable='turtle_tf2_listener',
         name='listener',
         parameters=[
            {'target_frame': LaunchConfiguration('target_frame')}
         ]
      ),
   ])
```

We can reuse the nodes by giving them different names. In the above launch file, we've declared a launch argument ```target_frame```. The argument would be passed to the node, and default value is set to 'turtle1'.

We can override the parameters. In the top level launch file, we've called the ```broadcaster_listener.launch.py``` file. We've passed that launch file ```target_frame``` launch argument as shown below:
```
broadcaster_listener_nodes = IncludeLaunchDescription(
   PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('launch_tutorial'), 'launch'),
      '/broadcaster_listener.launch.py']),
   launch_arguments={'target_frame': 'carrot1'}.items(),
   )
```
The default goal target frame is changed to ```carrot1```. If we want to use that default value ```turtle1```, just remove that line.

##### Remapping
Create a file ```mimic.launch.py```:
```
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      Node(
         package='turtlesim',
         executable='mimic',
         name='mimic',
         remappings=[
            ('/input/pose', '/turtle2/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
         ]
      )
   ])
```

This would start a node called mimic using an executable ```mimic``` in package ```turtlesim```. Then, we want to remap the topic ```/input/pose``` of mimic to ```/turtle2/pose``` of turtle2, and remap ```/output/cmd_vel``` of mimic to ```/turtlesim2/turtle1/cmd_vel``` of turtle1. This would pass the pose of turtle2, to the mimic node, and the mimic node would pass the message to the turtle1, resulting in turtle1 mimicing turtle2.

##### Config files
Create a file ```turtlesim_rviz.launch.py```:
```
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   rviz_config = os.path.join(
      get_package_share_directory('turtle_tf2_py'),
      'rviz',
      'turtle_rviz.rviz'
      )

   return LaunchDescription([
      Node(
         package='rviz2',
         executable='rviz2',
         name='rviz2',
         arguments=['-d', rviz_config]
      )
   ])
```
What is RViz? It's a 3d visualization tool for ROS. [About RViz](http://wiki.ros.org/rviz)

(quoted from tutorial) This launch file will start the RViz with the configuration file defined in the turtle_tf2_py package. This RViz configuration will set the world frame, enable TF visualization, and start RViz with a top-down view.

##### Environment Variables
Create a launch file ```fixed_broadcaster.launch.py```:
```
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable('USER'), '_'],
            description='prefix for node name'
      ),
      Node(
            package='turtle_tf2_py',
            executable='fixed_frame_tf2_broadcaster',
            name=[LaunchConfiguration('node_prefix'), 'fixed_broadcaster'],
      ),
   ])
```
> This launch file shows the way environment variables can be called inside the launch files. Environment variables can be used to define or push namespaces for distinguishing nodes on different computers or robots.

##### Run the launch files
Finally we can start running the launch files. Before we start, modify the setup.py file to include all the launch ```.py``` files and the configuration ```.yaml``` files:
```
import os
from glob import glob
from setuptools import setup
...

data_files=[
      ...
      (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
      (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
   ],
```

Then, build it, source it, and run it:
```
colcon build
source install/setup.bash
ros2 launch launch_tutorial launch_turtlesim.launch.py
```
(remember to download the rviz package):
```
sudo apt-get install ros-humble-rviz2
```
Here's the log message:
```
docker@ros2:~/ws/src/launch_ws$ ros2 launch launch_tutorial launch_turtlesim.launch.py
[INFO] [launch]: All log files can be found below /home/docker/.ros/log/2023-09-08-16-31-25-842726-ros2-982
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [turtlesim_node-1]: process started with pid [983]
[INFO] [turtlesim_node-2]: process started with pid [985]
[INFO] [turtle_tf2_broadcaster-3]: process started with pid [987]
[INFO] [turtle_tf2_broadcaster-4]: process started with pid [989]
[INFO] [turtle_tf2_listener-5]: process started with pid [991]
[INFO] [mimic-6]: process started with pid [993]
[INFO] [fixed_frame_tf2_broadcaster-7]: process started with pid [995]
[INFO] [rviz2-8]: process started with pid [997]
[ERROR] [rviz2-8]: process has died [pid 997, exit code 127, cmd '/opt/ros/humble/lib/rviz2/rviz2 -d /opt/ros/humble/share/turtle_tf2_py/rviz/turtle_rviz.rviz --ros-args -r __node:=rviz2'].
[rviz2-8] /opt/ros/humble/lib/rviz2/rviz2: error while loading shared libraries: libOgreMain.so.1.12.1: cannot open shared object file: No such file or directory
[turtlesim_node-1] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-docker'
[turtlesim_node-2] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-docker'
[turtlesim_node-1] [INFO] [1694161886.877540952] [sim]: Starting turtlesim with node name /sim
[turtlesim_node-2] [INFO] [1694161886.882499288] [turtlesim2.sim]: Starting turtlesim with node name /turtlesim2/sim
[turtlesim_node-2] [INFO] [1694161886.887794726] [turtlesim2.sim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
[turtlesim_node-1] [INFO] [1694161886.890579347] [sim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
[turtlesim_node-1] [INFO] [1694161888.724143457] [sim]: Spawning turtle [turtle2] at x=[4.000000], y=[2.000000], theta=[0.000000]
[turtle_tf2_listener-5] [INFO] [1694161889.716628642] [listener]: Successfully spawned turtle2
```
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-09-08%20163053.png)

We can see that the rviz failed launching, it is because we didn't source the setup file first. Let's try again:
```
docker@ros2:~/ws/src/launch_ws$ source install/setup.bash
docker@ros2:~/ws/src/launch_ws$ ros2 launch launch_tutorial launch_turtlesim.launch.py
[INFO] [launch]: All log files can be found below /home/docker/.ros/log/2023-09-08-16-34-58-646304-ros2-1128
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [turtlesim_node-1]: process started with pid [1129]
[INFO] [turtlesim_node-2]: process started with pid [1131]
[INFO] [turtle_tf2_broadcaster-3]: process started with pid [1133]
[INFO] [turtle_tf2_broadcaster-4]: process started with pid [1135]
[INFO] [turtle_tf2_listener-5]: process started with pid [1137]
[INFO] [mimic-6]: process started with pid [1139]
[INFO] [fixed_frame_tf2_broadcaster-7]: process started with pid [1141]
[INFO] [rviz2-8]: process started with pid [1143]
[turtlesim_node-1] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-docker'
[turtlesim_node-2] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-docker'
[rviz2-8] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-docker'
[turtlesim_node-2] [INFO] [1694162099.931566699] [turtlesim2.sim]: Starting turtlesim with node name /turtlesim2/sim
[turtlesim_node-1] [INFO] [1694162099.933387109] [sim]: Starting turtlesim with node name /sim
[turtlesim_node-1] [INFO] [1694162099.946094681] [sim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
[turtlesim_node-2] [INFO] [1694162099.947224387] [turtlesim2.sim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
[rviz2-8] [INFO] [1694162100.350545472] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-8] [INFO] [1694162100.350874073] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-8] [INFO] [1694162100.491061667] [rviz2]: Stereo is NOT SUPPORTED
[turtlesim_node-1] [INFO] [1694162101.868198567] [sim]: Spawning turtle [turtle2] at x=[4.000000], y=[2.000000], theta=[0.000000]
[turtle_tf2_listener-5] [INFO] [1694162102.858944977] [listener]: Successfully spawned turtle2
```
![](/ntu_racing_hw/ros2_setup/pictures/Screenshot%202023-09-08%20163537.png)

## That's all for the required parts of the ROS2 tutorials