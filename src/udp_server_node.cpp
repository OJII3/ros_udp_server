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

// Unityから送られてくるコントローラーの入力例
// (DuakShock3クラスから逆算している)
// c0000C00050050050
//
// data_head: c
// id: 000
// buttons(16進数): 0C00
// left_stick_x: 050 (0 ~ 100)
// left_stick_y: 050 (0 ~ 100)
// right_stick_x: 050 (0 ~ 100)
// right_stick_y: 050 (0 ~ 100)

int main(int argc, char **argv) {
  ros::init(argc, argv, "udp_server");

  const int local_port = 8888;
  const int target_port = 8888;
  auto topic_poleID = "poleID";
  auto topic_serial = "serial";

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
    recv_str = trim_right_copy(recv_str); // 末尾の空白/改行を削除

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

        ROS_INFO("Publishing to %s: %s", topic_serial, msg.data);
        serial_pub.publish(msg);

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
