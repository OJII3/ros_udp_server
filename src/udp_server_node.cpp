// TODO: 受信も送信もするので、udp_client みたいな名前にしたい

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

using namespace std;
using namespace boost::asio::ip;

template <typename To, typename From> To bit_cast(const From &from) noexcept {
  To result;
  std::memcpy(&result, &from, sizeof(To));
  return result;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "udp_server");

  constexpr int local_port = 8888;
  constexpr int target_port = 8888;
  auto topic_poleID = "poleID";
  auto topic_serial = "serial_joycon";

  boost::asio::io_service io_service;
  io_service.run();

  // 初回受信以降、スマホコントローラーのIPアドレス・ポートを保持する
  udp::endpoint remote_endpoint;

  // UDPソケットの作成
  udp::socket receive_socket(io_service, udp::endpoint(udp::v4(), local_port));
  udp::socket send_socket(io_service, remote_endpoint);
  ROS_INFO("UDP server started on localhost:%d", local_port);

  // 射出機構のノード用の Publisher, Subscriberの設定
  ros::NodeHandle nh;
  ros::Publisher publisher = nh.advertise<std_msgs::Float32>(topic_poleID, 10);
  ros::Subscriber subscriber = nh.subscribe<std_msgs::Float32>(
      topic_poleID, 10, [&](const std_msgs::Float32::ConstPtr &msg) -> void {
        // subscribeしたメッセージを送信するコールバック関数
        std::string data("%s", msg->data);
        send_socket.send_to(boost::asio::buffer(data), remote_endpoint);
      });

  // マイコンと通信するノード用の Publisherの設定
  ros::NodeHandle serial_nh;
  ros::Publisher serial_pub =
      serial_nh.advertise<std_msgs::String>(topic_serial, 100);

  // Ctrl+Cで終了するための設定
  boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);
  signals.async_wait([&](const boost::system::error_code &error, int signal) {
    if (!error) {
      io_service.stop();
      send_socket.shutdown(send_socket.shutdown_both);
      receive_socket.shutdown(receive_socket.shutdown_both);
    }
  });

  while (ros::ok()) {
    boost::array<uint8_t, 256> receive_byte_arr;
    receive_socket.receive_from(boost::asio::buffer(receive_byte_arr),
                                remote_endpoint);
    auto receive_char_arr = bit_cast<std::array<char, 256>>(receive_byte_arr);

    // 改行コードが一緒に送られてきても放置する

    if (fd == -1)
      ROS_INFO("fd is -1, usb port is not open");
    if (receive_byte_arr.size() > 0) {

      if (receive_byte_arr[0] == 74) {
        // check if first byte is "J"(74), which means Joycon
        // you should comment out the following line for better performance
        ROS_INFO("Joycon: %d, %d, %d, %d, %d, %d, %d, %d, %d",
                 receive_byte_arr[0], receive_byte_arr[1], receive_byte_arr[2],
                 receive_byte_arr[3], receive_byte_arr[4], receive_byte_arr[5],
                 receive_byte_arr[6], receive_byte_arr[7], receive_byte_arr[8]);

        std_msgs::ByteMultiArray msg;
        msg.data.resize(receive_byte_arr.size() - 1);
        for (int i = 1; i < receive_byte_arr.size(); i++) {
          msg.data[i] = receive_byte_arr[i];
        }
        // "serial_joycon"というトピック名でメッセージをpubする
        serial_pub.publish(msg);

      } else if (receive_byte_arr[0] == 80) {
        // check if first byte is "P"(80), which means Pole
        ROS_INFO("Pole: %d", receive_byte_arr[1]);

        std_msgs::Float32 msg;
        msg.data = receive_byte_arr[1];
        // "poleID"というトピック名でメッセージをpubする
        publisher.publish(msg);

      } else {
        ROS_INFO("Received unknown message");
      }
    }

    ros::spinOnce();
  }

  io_service.stop();
  receive_socket.shutdown(receive_socket.shutdown_both);
  send_socket.shutdown(send_socket.shutdown_both);

  return 0;
}
