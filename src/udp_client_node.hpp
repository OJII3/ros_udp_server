#pragma once
#include <array>
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/address.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <queue>
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

// 3 parts:
// to joycon(udp): subscribe to topic_toJoycon
// from joycon(udp): publish to topic_toJoycon
// from joycon(udp): publish to topic_poleID
class UDPClient {
public:
  // initialize node handle
  UDPClient();
  ~UDPClient();

  // start udp server
  void startUDPServer(const int &local_port);
  // start publishers and subscribers
  void startPubSub(const std::string &topic_poleID,
                   const std::string &topic_toUSB,
                   const std::string &topic_toJoycon);
  void sendToPoleIDNode(const uint8_t &data);
  void sendToUSBNode(const std::vector<uint8_t> &data);
  void sendToJoycon(const std_msgs::ByteMultiArray::ConstPtr &msg);
  std::vector<uint8_t> receiveFromJoycon();

private:
  ros::NodeHandle nh;

  ros::Publisher pub_poleID;
  ros::Publisher pub_toUSB;
  ros::Subscriber sub_joycon;

  boost::asio::ip::udp::endpoint joycon_endpoint;
  boost::asio::io_service io_service;
  boost::asio::ip::udp::socket send_socket;
  boost::asio::ip::udp::socket receive_socket;
}
