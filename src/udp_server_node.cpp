#include <ros/ros.h>
#include <boost/asio.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_server_node");
    ros::NodeHandle nh("~");

    std::string host = nh.param<std::string>("host", "localhost");
    int port = nh.param<int>("port", 5000);

    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket(io_service);
    boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address::from_string(host), port);
    socket.open(endpoint.protocol());
    socket.bind(endpoint);

    ROS_INFO("UDP server started on %s:%d", host.c_str(), port);

    boost::array<char, 1024> recv_buf;
    boost::asio::ip::udp::endpoint remote_endpoint;
    while (ros::ok())
    {
      boost::system::error_code error;
      size_t len = socket.receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0, error);
      if (error) {
          ROS_ERROR("UDP receive error: %s", error.message().c_str());
          continue; // ループを継続して受信を続ける
      }
      ROS_INFO("Received data from %s:%d: %s", remote_endpoint.address().to_string().c_str(), remote_endpoint.port(), recv_buf.data());

    }

    ros::spin();

    return 0;
}
