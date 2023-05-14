#include "udp_server_node.hpp"

using namespace boost::asio;
using namespace boost::asio::ip;

template <typename To, typename From>
To bit_cast(const From &from) noexcept
{
    To result;
    std::memcpy(&result, &from, sizeof(To));
    return result;
}

UDPServer::~UDPServer()
{
    send_socket.close();
    receive_socket.close();
    io_service.stop();
    pub_poleID.shutdown();
    pub_toUSB.shutdown();
    sub_controller.shutdown();
}

void UDPServer::startListening(const int &local_port)
{
    send_socket = udp::socket(io_service.run());
    receive_socket = udp::socket(io_service.run(), udp::endpoint(udp::v4(), local_port));
    bool is_connected = false;

    // wait for controller to connect, then joycon_endpoint will be set
    while (!is_connected)
    {
        std::vector<uint8_t> received_bytes = receiveFromController();
        if (received_bytes[0] == 74)
        { //"J"
            is_connected = true;
        }
    }
}

void UDPServer::startRosNode(const std::string &topic_poleID,
                             const std::string &topic_toUSB,
                             const std::string &topic_toController)
{
    nh = ros::NodeHandle("~");
    pub_poleID = nh.advertise<std_msgs::Float32>(topic_poleID, 1);
    pub_toUSB = nh.advertise<std_msgs::ByteMultiArray>(topic_toUSB, 1);
    sub_controller = nh.subscribe<std_msgs::ByteMultiArray>(
        topic_toController, 10,
        [&](const std_msgs::ByteMultiArray::ConstPtr &msg) -> void
        {
            sendToController(msg);
        });
}

void UDPServer::sendToPoleIDNode(const uint8_t &data)
{
    std_msgs::Float32 msg;
    msg.data = data;
    pub_poleID.publish(msg);
}

void UDPServer::sendToUSBNode(const std::vector<uint8_t> &data)
{
    std_msgs::ByteMultiArray msg;
    for (int i = 0; i < data.size(); i++)
    {
        msg.data.push_back(data[i]);
    }
    pub_toUSB.publish(msg);
}

void UDPServer::sendToController(const std_msgs::ByteMultiArray::ConstPtr &msg)
{
    std::array<uint8_t, 16> send_bytes;
    for (int i = 0; i < msg->data.size(); i++)
    {
        send_bytes[i] = msg->data[i];
    }
    send_socket.send_to(boost::asio::buffer(send_bytes), controller_endpoint);
}

std::vector<uint8_t> UDPServer::receiveFromController()
{
    std::array<uint8_t, 16> received_bytes;
    receive_socket.receive_from(
        buffer(received_bytes), controller_endpoint);
    return bit_cast<std::vector<uint8_t>>(received_bytes);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_server_node");
    const int local_port = 8888;
    const std::string topic_poleID = "poleID";
    const std::string topic_toUSB = "usb_packet_tomicon";
    const std::string topic_toController = "usb_to_controller";

    try
    {
        ROS_INFO("UDP client node started");
        UDPServer udp_server;

        // startUDPServer must be called before startPubSub
        udp_server.startListening(local_port);
        udp_server.startRosNode(topic_poleID, topic_toUSB, topic_toController);

        while (ros::ok())
        {
            auto received_bytes = udp_server.receiveFromController();

            ROS_INFO("Received %d bytes, [%d, %d, %d, %d, %d, %d, %d, %d, ...]",
                     bit_cast<uint8_t>(received_bytes.size()), received_bytes[0],
                     received_bytes[1], received_bytes[2], received_bytes[3],
                     received_bytes[4], received_bytes[5], received_bytes[6],
                     received_bytes[7]);

            if (received_bytes[0] == 80)
            { //"P"
                udp_server.sendToPoleIDNode(received_bytes[1]);
            }
            else if (received_bytes[0] == 74)
            { //"J"
                // remove first byte
                received_bytes.erase(received_bytes.begin());
                udp_server.sendToUSBNode(received_bytes);
            }
            else
            {
                ROS_ERROR("Received unknown packet");
            }

            ros::spinOnce();
        }
    }
    catch (std::exception &e)
    {
        ROS_INFO("Exception: %s", e.what());
    }

    ros::shutdown();

    return 0;
}
