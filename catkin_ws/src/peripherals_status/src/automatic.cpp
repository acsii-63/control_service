#include "peripherals_status/peripherals_status.h"
#include "/home/pino/pino_ws/papi/PAPI.h"

PAPI::communication::Server server(DEFAULT_PERIPHERALS_STATUS_NODE_PORT);
PAPI::communication::Client client(LOCAL_HOST, DEFAULT_PERIPHERALS_STATUS_CONTROL_PORT);

std::vector<int> list;

int connectToControlService()
{
    return (server.serverStart() == -1 || client.clientStart() == -1) ? -1 : 0;
}

void addPeripherals()
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
        msg_str = client.reciveMessage();

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

    // std::vector<int> list{DEVICE::FLIR, DEVICE::D455, DEVICE::T265};
    addPeripherals();
    peripherals_status->addPeripherals(list);

    while (ros::ok())
    {
        // peripherals_status->debug();

        peripherals_status->callBack_exist();

        server.sendMsg(peripherals_status->getStatus_toString());
        
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    closeSocket();

    return 0;
}