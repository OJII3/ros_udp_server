# ROS1 UDP Server Package

## Envitonment

- `Ubuntu`: v22.04
- `Docker`(Base image `ros:noetic-robot`)
- `catkin`: v0.9.2
- `ROS`(noetic): v1.16.0

(In my opinion,) Ubuntu20.04 is the much better environment.

## Summary

uint8_t の配列を UDP 通信で受け取る。UDP ポートは 8888。

改行コードなどが含まれていても放置する。

### ジョイコン(DualShock3)の入力

スマホ(Unity)から受信

```cpp
["J"(=74), id(1), ボタンの状態(0 or 1)*4, ボタンの状態*4, ボタンの状態*4, ボタンの状態*4, leftPad_x(-50~50), leftPad_y, rightPad_x, rightPad_y]
```

ros_topic "serial_joycon" に送信(`std_msgs::byteMultiArray`)

```cpp
[id(1), ボタンの状態(0 or 1)*4, ボタンの状態*4, ボタンの状態*4, ボタンの状態*4, leftPad_x(-50~50), leftPad_y, rightPad_x, rightPad_y]
```

(`feature/usb` ブランチでは、USB で直接マイコンに送りつける機能もつけている。その際、どの USB ポートを使っているか調べ、ソースコードを修正,`sudo chmod 666 /path/to/usb` で権限をあたえてから)

### ポールの番号

スマホ(Unity)から受信

```cpp
["P"(=80), ポールの番号(1~13)]
```

ros_topic "poleID" に送信(`std_msgs::Float32`)

```cpp
ポールの番号(1~13)
```

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

4. Check the connection

- Test with `netcat`(use another device)

```shell
nc -u <your-ip-address> 8888
```

- Send from Unity Application(use another device)

Please find my unity project from my github | gitlab repository.

## What I did for preparation - 1/2 (for WSL2 Environment)

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

Then edit `udp_server_node.cpp`, `MakeLists.txt`, and `package.xml`.

This time, I used `Boost` Library for udp connection.

## What I did for preparation - 2/2 (for Docker Environment)

Basic Envrionment: Ubuntu22.04

### Install Docker

You can do it by yourself. I just installed Docker Desktop, and didn't do anything for DockerHub.

### Pull the ROS Image and Run to Create Container

```shell
docker pull ros:noetic-robot
```

```shell
docker run -it --name ros-noetic -p 8888:8888/udp 11311:11311 ros:noetic-robot
```

You need to forward some ports so that you can use the host's ports in the container.

In this case, you need to use port 8888 for udp server, and 11311 for defalut ros Master.

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

## References

- [Linux でシリアル通信をしようとした話 - Qiita](https://qiita.com/sttn/items/567c9f49b88ff275b51a)
