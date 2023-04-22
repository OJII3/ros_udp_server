#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/address.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sys/socket.h>

using boost::asio::ip::udp;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_server");
    
    int local_port = 8888;
    int target_port = 8888;

    boost::asio::io_service io_service;
    io_service.run();

    // 初回受信以降、スマホコントローラーのIPアドレス・ポートを保持する
    udp::endpoint remote_endpoint;

    udp::socket receive_socket(io_service, udp::endpoint(udp::v4(), local_port));
    udp::socket send_socket(io_service, remote_endpoint);

    ROS_INFO("UDP server started on localhost:%d", local_port);


    auto topic_name = "conttoller";
    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<std_msgs::Float32>(topic_name, 10);
    ros::Subscriber subscriber = nh.subscribe<std_msgs::Float32>(topic_name, 10, [&](const std_msgs::Float32::ConstPtr& msg) -> void {
      std::string data("%s", msg->data);
      send_socket.send_to(boost::asio::buffer(data), remote_endpoint);
    });
    
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
        // udpの受信
        boost::array<char, 1024> recv_buf;
        size_t len = receive_socket.receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0);
        std::string recv_str(recv_buf.data(), len);

        ROS_INFO("Received data from %s:%d: %s", remote_endpoint.address().to_string().c_str(), remote_endpoint.port(), recv_str.c_str());

        // ros topic へ publish
        std_msgs::Float32 msg;
        std::istringstream iss(recv_str.c_str());
        int int_msg;
        while(iss >> int_msg) {
            msg.data = int_msg;
        }
        ROS_INFO("Publishing: %f", msg.data);
        publisher.publish(msg);

        receive_socket.send_to(boost::asio::buffer(recv_str), remote_endpoint);
    
        ros::spinOnce();
    }

    io_service.stop();
    receive_socket.shutdown(receive_socket.shutdown_both);
    send_socket.shutdown(send_socket.shutdown_both);

    return 0;
}
