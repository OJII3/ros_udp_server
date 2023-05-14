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
// to controller(udp): subscribe to topic_toController
// from controller(udp): publish to topic_toController
// from controller(udp): publish to topic_poleID
// TODO: more abstraction
class UDPServer {
public:
  // initialize the node handle
  UDPServer();
  ~UDPServer();

  // start udp server
  void startListening(const int &local_port);
  // start publishers and subscribers
  void startRosNode(const std::string &topic_poleID,
                   const std::string &topic_toUSB,
                   const std::string &topic_toController);
  void sendToPoleIDNode(const uint8_t &data);
  void sendToUSBNode(const std::vector<uint8_t> &data);
  void sendToController(const std_msgs::ByteMultiArray::ConstPtr &msg);
  std::vector<uint8_t> receiveFromController();

private:
  ros::NodeHandle nh;

  ros::Publisher pub_poleID;
  ros::Publisher pub_toUSB;
  ros::Subscriber sub_controller;

  boost::asio::ip::udp::endpoint controller_endpoint;
  boost::asio::io_service io_service;
  boost::asio::ip::udp::socket send_socket;
  boost::asio::ip::udp::socket receive_socket;
};
