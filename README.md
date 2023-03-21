# ROS1 UDP Server Package

## Envitonment

- `Ubuntu`(wsl2): v20.04
- `catkin`: v0.9.2
- `ROS`(noetic): v1.16.0

## Aim

- Receive text messages Sent via UDP
- Log the received text

**Since it only has minimal functionality for receiving and logging output, significant modifications may be required depending on the requirements.**

## How to use this

1. Clone this repository to your `catkin_ws` and move to the root directory.

```shell
git clone <git-repository-url> udp_server
cd udp_server
```

2. Build

```shell
catkin build --this
```

3. Run UDP Server

First, you have to check and remember your IP address(v4).

```shell
hostname -I
```

Open anorher terminal and run the Master.

```shell
roscore
```

While the Master is running, start the node.

```shell
rosrun udp_server udp_server_node
```

Now, your udp server is listening on `localhost:8888`!


4. Send Text to the Server

Here is an example of sending text from CLI. Open another terminal.

```shell
nc -u <target-ip-address> 8888  #use netcat to establish a connection
```

When you type some text and press the <key>Enter</key>, a log will appear on your terminal that runs udp server.

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

```shell
vim ~/.bashrc 
```

And edit like this.

```diff
+ source /opt/ros/noetic/setup.bash
+ source ~/src/catkin_ws/devel/setup.bash
```

Then, install urg-node

```shell
sudo apt install ros-noetic-urg-node
```

### Create Package

```shell
catkin create udp_server
cd udp_server
mkdir src
```
Then edit `udp_server_node.cpp` and `MakeLists.txt`.

```shell
vim .
```

This time, I used `Boost` Library.

