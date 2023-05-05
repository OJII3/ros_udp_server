#include <array>
#include <boost/algorithm/string/trim.hpp>
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
#include <string>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

using namespace std;
using namespace boost::asio::ip;
using namespace boost::algorithm;

template <typename To, typename From> To bit_cast(const From &from) noexcept {
  To result;
  std::memcpy(&result, &from, sizeof(To));
  return result;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "udp_server");

  const int local_port = 8888;
  const int target_port = 8888;
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

  // ポール用の Publisher, Subscriberの設定
  ros::NodeHandle nh;
  ros::Publisher publisher = nh.advertise<std_msgs::Float32>(topic_poleID, 10);
  ros::Subscriber subscriber = nh.subscribe<std_msgs::Float32>(
      topic_poleID, 10, [&](const std_msgs::Float32::ConstPtr &msg) -> void {
        // subscribeしたメッセージを送信するコールバック関数
        std::string data("%s", msg->data);
        send_socket.send_to(boost::asio::buffer(data), remote_endpoint);
      });

  // USBシリアル通信を行うインターフェース用のROSノードとの接続用の設定
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
    // udpの受信
    boost::array<uint8_t, 256> receive_byte_arr;
    receive_socket.receive_from(boost::asio::buffer(receive_byte_arr),
                                remote_endpoint);
    auto receive_char_arr = bit_cast<std::array<char, 256>>(receive_byte_arr);
    auto receive_str =
        std::string(std::begin(receive_char_arr), std::end(receive_char_arr));

    receive_str = trim_right_copy(receive_str); // 末尾の空白/改行を削除

    if (receive_byte_arr.size() > 0) {

      ROS_INFO("Received data from %s:%d: %s",
               remote_endpoint.address().to_string().c_str(),
               remote_endpoint.port(), receive_str.c_str());

      ROS_INFO("publihing: %d, %d, %d, %d, %d, %d, %d, %d, %d",
               receive_byte_arr[0], receive_byte_arr[1], receive_byte_arr[2],
               receive_byte_arr[3], receive_byte_arr[4], receive_byte_arr[5],
               receive_byte_arr[6], receive_byte_arr[7], receive_byte_arr[8]);

      // check if first byte encoded is "J"
      if (receive_byte_arr[0] == 74) {
        // if message is joystick input, write to USB serial

        std_msgs::ByteMultiArray msg;
        msg.data.resize(receive_byte_arr.size() - 1);
        for (int i = 1; i < receive_byte_arr.size(); i++) {
          msg.data[i] = receive_byte_arr[i];
        }

        serial_pub.publish(msg);
      } else if (receive_str.substr(0, 2) == "P.") {
        // if message is number(poleID), publish to ros topic

        std_msgs::Float32 msg;
        std::istringstream iss(receive_str.substr(2).c_str());
        int int_msg;
        while (iss >> int_msg) {
          msg.data = int_msg;
        }
        // ros topic へ publish
        publisher.publish(msg);
      } else {
        ROS_INFO("Received unknown message");
      }
    }

    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  io_service.stop();
  receive_socket.shutdown(receive_socket.shutdown_both);
  send_socket.shutdown(send_socket.shutdown_both);

  return 0;
}
