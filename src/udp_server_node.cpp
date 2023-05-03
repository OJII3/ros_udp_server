#pragma once
#include <boost/algorithm/string/trim.hpp>
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/address.hpp>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <queue>
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <string>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

using namespace std;
using namespace boost::asio::ip;
using namespace boost::algorithm;

int openUSBSerial(int &fd) {
  // char device_name[] = "/dev/ttyUSB0"; / /UART用
  char device_name[] = "/dev/ttyACM0"; // MasterBoard
  fd = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
  fcntl(fd, F_SETFL, 0);
  // load conf"iguration
  struct termios conf_tio;
  tcgetattr(fd, &conf_tio);
  // set baudrate
  speed_t BAUDRATE = B115200;
  cfsetispeed(&conf_tio, BAUDRATE);
  cfsetospeed(&conf_tio, BAUDRATE);

  // make raw mode
  cfmakeraw(&conf_tio); // <<<<<<<<<<<<<

  // non canonical, non echo back
  conf_tio.c_lflag &= ~(ECHO | ICANON);
  // non blocking
  conf_tio.c_cc[VMIN] = 0;
  conf_tio.c_cc[VTIME] = 0;
  // store configuration
  tcsetattr(fd, TCSANOW, &conf_tio);
  // std::cerr<<"OPENED"<<std::endl;
  std::cerr << "fd = " << fd << std::endl;
  if (fd > 0)
    std::cerr << "OPENED" << std::endl;
  else
    std::cerr << "CANNOT OPEN" << std::endl;
  return fd;
}

uint8_t parseGamePadInput(std::string inputdata) {
  // C++ かけないのでクソコードだけどゆるして
  // Unity側で定義したボタンの名前を入力すると、そのボタンの値を返す(やはりクソせっけい)
  if (inputdata == "R3")
    return uint8_t(0x0001);
  else if (inputdata == "L3")
    return uint8_t(0x0002);
  else if (inputdata == "Selct")
    return uint8_t(0x0004);
  else if (inputdata == "Start")
    return uint8_t(0x0008);
  else if (inputdata == "R2")
    return uint8_t(0x0010);
  else if (inputdata == "L2")
    return uint8_t(0x0020);
  else if (inputdata == "R1")
    return uint8_t(0x0040);
  else if (inputdata == "L1")
    return uint8_t(0x0080);
  else if (inputdata == "Square")
    return uint8_t(0x0100);
  else if (inputdata == "Cross")
    return uint8_t(0x0200);
  else if (inputdata == "Circle")
    return uint8_t(0x0400);
  else if (inputdata == "Triangle")
    return uint8_t(0x0800);
  else if (inputdata == "DPadLeft")
    return uint8_t(0x1000);
  else if (inputdata == "DPadDown")
    return uint8_t(0x2000);
  else if (inputdata == "DPadRight")
    return uint8_t(0x4000);
  else if (inputdata == "DPadUp")
    return uint8_t(0x8000);
  else
    return uint8_t(0x0000);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "udp_server");

  // USBシリアルポートのオープン
  int fd;
  fd = openUSBSerial(fd);

  const int local_port = 8888;
  const int target_port = 8888;

  boost::asio::io_service io_service;
  io_service.run();

  // 初回受信以降、スマホコントローラーのIPアドレス・ポートを保持する
  udp::endpoint remote_endpoint;

  // UDPソケットの作成
  udp::socket receive_socket(io_service, udp::endpoint(udp::v4(), local_port));
  udp::socket send_socket(io_service, remote_endpoint);
  ROS_INFO("UDP server started on localhost:%d", local_port);

  // Publisher, Subscriberの設定
  auto topic_poleID = "poleID";
  ros::NodeHandle nh;
  ros::Publisher publisher = nh.advertise<std_msgs::Float32>(topic_poleID, 10);
  ros::Subscriber subscriber = nh.subscribe<std_msgs::Float32>(
      topic_poleID, 10, [&](const std_msgs::Float32::ConstPtr &msg) -> void {
        std::string data("%s", msg->data);
        send_socket.send_to(boost::asio::buffer(data), remote_endpoint);
      });

  // Ctrl+Cを受信したら終了する
  boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);
  signals.async_wait(
      [&io_service](const boost::system::error_code &error, int signal_number) {
        if (!error) {
          io_service.stop();
        }
      });

  while (ros::ok()) {
    // udpの受信
    boost::array<char, 1024> recv_buf;
    size_t len = receive_socket.receive_from(boost::asio::buffer(recv_buf),
                                             remote_endpoint, 0);
    std::string recv_str(recv_buf.data(), len);
    recv_str = trim_right_copy(recv_str);

    if (recv_str.length() > 0) {

      ROS_INFO("Received data from %s:%d: %s",
               remote_endpoint.address().to_string().c_str(),
               remote_endpoint.port(), recv_str.c_str());

      if (recv_str.substr(0, 1) == "c") {
        // if message is joystick input, write to USB serial
        stds_msgs::MultiByteArray msg;
        msg.data.resize(recv_str.length());
        for (int i = 0; i < recv_str.length(); i++) {
          msg.data[i] = recv_str[i];
        }

        char start = 'SS';
        char end = 'EE';
        char buf[sizeof(char) + sizeof(uint8_t) + sizeof(char)];
        memcpy(buf, &start, sizeof(char));
        memcpy(buf + sizeof(char), &data, sizeof(data));
        memcpy(buf + sizeof(char) + sizeof(uint8_t), &end, sizeof(char));
        ROS_INFO("fd: %d", fd);
        auto n = write(fd, buf, sizeof(buf));
      } else if (recv_str.substr(0, 2) == "P.") {
        // if message is number(poleID), publish to ros topic
        std_msgs::Float32 msg;
        std::istringstream iss(recv_str.substr(2).c_str());
        int int_msg;
        while (iss >> int_msg) {
          msg.data = int_msg;
        }
        // ros topic へ publish
        publisher.publish(msg);
      }
    }

    ros::spinOnce();
  }

  io_service.stop();
  receive_socket.shutdown(receive_socket.shutdown_both);
  send_socket.shutdown(send_socket.shutdown_both);

  return 0;
}
