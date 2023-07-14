#include "control_pkg/control.h"
#include "/home/pino/pino_ws/papi/PAPI.h"

PAPI::communication::Server server(DEFAULT_STATUS_ROS_NODE_PORT);
PAPI::communication::Client client(LOCAL_HOST, DEFAULT_STATUS_CONTROL_SERVICE_PORT);

std::vector<int> list;
std::string id;

int connectToControlService()
{
    int serverResult = server.serverStart();
    PAPI::system::sleepLessThanASecond(0.5);
    int clientResult = client.clientStart();

    if (serverResult == -1 || clientResult == -1)
        return -1;

    return 0;
}

void listener()
{
    list.push_back(DEVICE::FCU_BAT);
    list.push_back(DEVICE::FCU_GPS);
    list.push_back(DEVICE::FCU_IMU);
    list.push_back(DEVICE::FCU_MAG);
    list.push_back(DEVICE::FCU_ODOM);
    list.push_back(DEVICE::FCU_PRES);
    list.push_back(DEVICE::FCU_STATE);

    std::string msg_str = "";
    while (msg_str.empty())
        msg_str = client.receiveMessage();

    std::stringstream ss(msg_str);
    std::string number_str;
    while (std::getline(ss, number_str, '|'))
    {
        int num;
        try
        {
            num = std::stoi(number_str);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        switch (num)
        {
        case Peripheral::PERIPHERAL_CAM_DOWNWARD:
            list.push_back(DEVICE::FLIR);
            break;

        case Peripheral::PERIPHERAL_CAM_FORWARD:
            list.push_back(DEVICE::D455);
            break;

        case Peripheral::PERIPHERAL_CAM_ODOM:
            list.push_back(DEVICE::T265);
            break;

        case Peripheral::PERIPHERAL_LIDAR:
            list.push_back(DEVICE::LIDAR);
            break;

        case Peripheral::PERIPHERAL_RANGE_FINDER:
            list.push_back(DEVICE::TERABEE);
            break;

        default:
            break;
        }
    }

    msg_str.clear();
    while (msg_str.empty())
        msg_str = client.receiveMessage();
    id = std::string(msg_str);
}

void closeSocket()
{
    server.serverClose();
    client.clientClose();
}

int main(int argc, char **argv)
{
    if (connectToControlService() == -1)
    {
        std::cerr << "Cannot connect to control service.\n";
        return -1;
    }

    ros::init(argc, argv, "automatic");
    ros::NodeHandle nh("");
    PeripheralsStatus *peripherals_status = new PeripheralsStatus(nh);
    // RouteStatus *route_status = new RouteStatus(nh);

    listener();
    peripherals_status->addPeripherals(list);
    peripherals_status->addMissionID(id);
    peripherals_status->setImagePath();

    while (ros::ok())
    {
        peripherals_status->callBack_exist();

        server.sendMsg(peripherals_status->getStatus_toString());

        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    closeSocket();

    return 0;
}