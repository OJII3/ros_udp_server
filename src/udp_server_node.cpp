#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>

using boost::asio::ip::udp;

udp::endpoint 


// Declare callback functions for UDP message reception and ROS message subscription
void handle_receive(const boost::system::error_code& error,
                    std::size_t bytes_transferred,
                    udp::socket& socket,
                    udp::endpoint& sender_endpoint,
                    ros::Publisher& pub);
void handle_subscribe(const std_msgs::String::ConstPtr& );

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_ros_bridge");
    ros::NodeHandle nh;
    udp::endpoint sender_endpoint;
    udp::endpoint target_endpoint;

    // Subscribe to the ROS topic
    ros::Publisher pub = nh.advertise<std_msgs::String>("controller", 1000);
    ros::Subscriber sub = nh.subscribe<std_msgs::String>("controller", 1000, handle_subscribe);

    // Set up UDP socket
    boost::asio::io_service io_service;
    udp::socket socket(io_service, udp::endpoint(udp::v4(), 8888));

    // Receive UDP messages and publish them to ROS topic
    boost::array<char, 1024> recv_buffer;
    socket.async_receive_from(boost::asio::buffer(recv_buffer), sender_endpoint, 0);
    pub.publish(msg);

    // Start the IO service
    io_service.run();

    return 0;
}

void handle_receive(const boost::system::error_code& error,
                    boost::array<char, 1024>& recv_buffer,
                    std::size_t bytes_transferred,
                    udp::socket& socket,
                    udp::endpoint& sender_endpoint,
                    ros::Publisher& pub)
{
  if (!error || error == boost::asio::error::message_size)
  {
    // Get the target endpoint from the first received UDP message's IP
    std::string target_ip = sender_endpoint.address().to_string();
    udp::endpoint target_endpoint(boost::asio::ip::address::from_string(target_ip), 8888);

    // Publish the received message to the ROS topic
    std_msgs::String msg;
    msg.data = std::string(recv_buffer.data(), bytes_transferred);
    pub.publish(msg);

    // Send the ROS message to the target endpoint
    boost::asio::async_write(socket, boost::asio::buffer(msg.data),
        boost::bind(&[](const boost::system::error_code& error,
                        std::size_t bytes_transferred)
        {
          // Do nothing on send completion
        }));
  }

  // Continue receiving UDP messages
  socket.async_receive_from(boost::asio::buffer(recv_buffer),
      sender_endpoint,
      boost::bind(handle_receive, _1, _2, std::ref(socket), std::ref(sender_endpoint), std::ref(pub)));
}

void handle_subscribe(const std_msgs::String::ConstPtr& msg,
                      udp::socket& socket,
                      udp::endpoint& target_endpoint)
{
  // Send the ROS message to the target endpoint
  boost::asio::async_write(socket, boost::asio::buffer(msg->data),
      boost::bind(&[](const boost::system::error_code& error,
                      std::size_t bytes_transferred)
      {
        // Do nothing on send completion
      }));
}
