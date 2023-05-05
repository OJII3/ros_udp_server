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

int openUSBSerial() {
  // char device_name[] = "/dev/ttyUSB0"; / /UART用
  char device_name[] = "/dev/ttyACM0"; // MasterBoard
  int fd = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
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

  int fd = openUSBSerial();

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

        auto data = std::make_shared<std_msgs::ByteMultiArray>(msg);
        data.data.reserve(msg.data.size() + 5);
        data.data.push_back(start);
        data.data.push_back(start);
        data.data.push_back(topicIdentifier.joycon.first);
        data.data.insert(data.data.end(), msg.data.begin(), msg.data.end());
        data.data.push_back(end);
        data.data.push_back(end);

        write(fd, data->data.data(), data->data.size());

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
