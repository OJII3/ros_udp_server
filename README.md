# ROS1 UDP Server Package

## Envitonment

- `Ubuntu(wsl2)`: v20.04
- `catkin`: 

## How to use this

First of all, clone this repository and move to the root directory.

Then, build.

```shell
catkin build --this
```

## What I did

### Install ROS1

```shell
sudo apt install curl
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

### Install and Init catkin

```shell
sudo apt install python3-catkin-tools

source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin build
```

### Create Package

```shell
catkin create udp_server
cd udp_server
mkdir src
vim udp_server.cpp
```

Then edit `udp_server.cpp` like this.

```cpp
#include <ros/ros.h>
#include <boost/asio.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_server");
    ros::NodeHandle nh("~");

    std::string host;
    int port;
    nh.param<std::string>("host", host, "localhost");
    nh.param<int>("port", port, 5000);

    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket(io_service);
    boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address::from_string(host), port);
    socket.open(endpoint.protocol());
    socket.bind(endpoint);

    ROS_INFO("UDP server started on %s:%d", host.c_str(), port);

    boost::array<char, 1024> recv_buf;
    boost::asio::ip::udp::endpoint remote_endpoint;
    while (ros::ok())
    {
        boost::system::error_code error;
        size_t len = socket.receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0, error);
        if (error && error != boost::asio::error::message_size)
            throw boost::system::system_error(error);
        ROS_INFO("Received data from %s:%d: %s", remote_endpoint.address().to_string().c_str(), remote_endpoint.port(), recv_buf.data());
    }

    return 0;
}
```

Then, edit `MakeLists.txt`.

```shell
cd ..
vim MakeLists.txt
```

```txt
cmake_minimum_required(VERSION 2.8.3)
project(my_udp_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(my_udp_server src/my_udp_server.cpp)
target_link_libraries(my_udp_server
  ${catkin_LIBRARIES}
)
```

Then, build.

```shell
catkin build --this
```

Then, run

```shell
source ~/src/catkin_ws/devel/setup.bash
```
