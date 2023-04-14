#include <boost/asio/io_context.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/address.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_server");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<std_msgs::Float32>("controller", 10);
    
    int local_port = 8888;
    int target_port = 8888;

    boost::asio::io_service io_service;
    io_service.run();

    udp::socket udp_socket(io_service, udp::endpoint(udp::v4(), local_port));

    ROS_INFO("UDP server started on localhost:%d", local_port);

    udp::endpoint remote_endpoint;
    
    // Ctrl+Cを受信したら終了する
    boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);
    signals.async_wait([&io_service](const boost::system::error_code& error, int signal_number)
    {
        if (!error)
        {
            io_service.stop();
        }
    });

    while (ros::ok())
    {
      boost::array<char, 1024> recv_buf;
      size_t len = udp_socket.receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0);
      std::string recv_str(recv_buf.data(), len);

      ROS_INFO("Received data from %s:%d: %s", remote_endpoint.address().to_string().c_str(), remote_endpoint.port(), recv_str.c_str());

        try {
            std_msgs::Float32 msg;
            std::stringstream ss;
            ss << recv_str;
            msg.data = std::stof(ss.str());
            publisher.publish(msg);
            udp_socket.send_to(boost::asio::buffer(recv_str), remote_endpoint);
        } catch (char* e) {
            udp_socket.send_to(boost::asio::buffer("Error: Please send Message that can be converted to foat."), remote_endpoint);
        }
        
        ros::spinOnce();
    }

    return 0;
}