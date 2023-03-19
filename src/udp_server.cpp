#include <ros/ros.h>
#include <boost/asio.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_server");
    ros::NodeHandle nh("~");

    std::string host;
    int port;
    nh.param<std::string>("host", host, "localhost");
    nh.param<int>("port", port, 5000);

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
        if (error && error != boost::asio::error::message_size)
            throw boost::system::system_error(error);
        ROS_INFO("Received data from %s:%d: %s", remote_endpoint.address().to_string().c_str(), remote_endpoint.port(), recv_buf.data());
    }

    return 0;
}
