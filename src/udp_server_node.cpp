#include <boost/asio/io_context.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/address.hpp>
#include <ros/ros.h>
#include <boost/asio.hpp>


using boost::asio::ip::udp;

int main()
{
    // ros::init(argc, argv, "udp_server_node");
    // ros::NodeHandle nh("~");
    
    int port = 8888;

    boost::asio::io_service io_service;
    io_service.run();

    udp::socket sock(io_service, udp::endpoint(udp::v4(), port));

    ROS_INFO("UDP server started on localhost:%d", port);

    udp::endpoint remote_endpoint;

    while (true)
    {
      boost::array<char, 1024> recv_buf;
      size_t len = sock.receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0);
      ROS_INFO("Received data from %s:%d: %s", remote_endpoint.address().to_string().c_str(), remote_endpoint.port(), recv_buf.data());

    }

    return 0;
}
