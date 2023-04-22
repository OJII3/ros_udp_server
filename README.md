# ROS1 UDP Server Package

## Envitonment

- `Ubuntu`: v22.04
- `Docker`(Base image `ros:noetic-robot`)
- `catkin`: v0.9.2
- `ROS`(noetic): v1.16.0

(In my opinion,) Ubuntu20.04 is the much better environment.

## Aim

- [x] Receive UDP messages from a smartphone (output to log)
- [x] Publish the received message to the ros topic "controller"
- [x] Subscribe the topic "controller" and send the message to the smartphone
- [x] Custmoize the type of pub/sub messages

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

The last goal is to send data from a smartphone. The smartphone app's repository is [here](https://github.com/ojii3/udp_controller_unity).

## What I did - 1/2 (for WSL2 Environment)

Basic Environment: Windows Subsystem for Linux 2 (wsl2), Ubuntu20.04

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
Then edit `udp_server_node.cpp`, `MakeLists.txt`, and  `package.xml`.

This time, I used `Boost` Library for udp connection.

## What I did - 2/2 (for Docker Environment)

Basic Envrionment: Ubuntu22.04

### Install Docker

You can do it by yourself. I just installed Docker Desktop, and didn't do anything for DockerHub.

### Pull the ROS Image and Run to Create Container

```shell
docker pull ros:noetic-robot
```

```shell
docker run -it --name ros-noetic -p 8888:8888/udp -p 11311:11311 ros:noetic-robot
```

You need forward some ports. In this case you use port 8888 to receive udp message, and port 11311 for ros Master.

### Install tools

```shell
apt update
apt-get update
apt-get install git
apt-get install python3-catkin-tools
apt-get install python3-rosnode
```

You should install other tools if you need, such as `vim`, `netcat`, `curl` etc.

Other processes are the same as the WSL2 environment, but you have to setup git or ssh beforehands to develop on the container.
