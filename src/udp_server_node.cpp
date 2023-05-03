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
        std_msgs::ByteMultiArray msg;
        msg.data.resize(recv_str.length());
        for (int i = 0; i < recv_str.length(); i++) {
          msg.data[i] = recv_str[i];
        }

        constexpr char startChar = 'S';
        constexpr char endChar = 'E';
        constexpr uint8_t start = static_cast<uint8_t>(startChar);
        constexpr uint8_t end = static_cast<uint8_t>(endChar);

        struct TopicIdentifier {
          std::pair<uint8_t, std::string> launcher;
          std::pair<uint8_t, std::string> path;
          std::pair<uint8_t, std::string> joycon;
        };

        auto topicIdentifier = TopicIdentifier{
            {0x01, "launcher"}, {0x02, "path"}, {0x03, "joycon"}};

        std_msgs::ByteMultiArrayPtr data =
            boost::make_shared<std_msgs::ByteMultiArray>();
        data->data.reserve(msg->data.size() + 5);
        data->data.push_back(start);
        data->data.push_back(start);
        data->data.push_back(topicIdentifier.first);
        data->data.push_back(data->data.end(), msg->data.begin(),
                             msg->data.end());
        data->data.push_back(end);
        data->data.push_back(end);

        write(fd, data->data.data(), data->data.size());

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
