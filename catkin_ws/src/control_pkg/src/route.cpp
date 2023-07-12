#include "control_pkg/control.h"
#include "/home/pino/pino_ws/papi/PAPI.h"

// PAPI::communication::Server server(DEFAULT_ROUTE_ROS_NODE_PORT);
// PAPI::communication::Client client(LOCAL_HOST, DEFAULT_ROUTE_CONTROL_SERVICE_PORT);

// int connectToControlService()
// {
//     int serverResult = server.serverStart();
//     int clientResult = client.clientStart();

//     if (serverResult == -1 || clientResult == -1)
//         return -1;

//     return 0;
// }

// void closeSocket()
// {
//     server.serverClose();
//     client.clientClose();
// }

int main(int argc, char **argv)
{
    // if (connectToControlService() == -1)
    // {
    //     std::cerr << "Cannot connect to control service.\n";
    //     return -1;
    // }

    ros::init(argc, argv, "route");
    ros::NodeHandle nh("");
    RouteStatus *route_status = new RouteStatus(nh);

    while (ros::ok())
    {
        // server.sendMsg(route_status->routeMessageFoward());
        PAPI::communication::sendMessage_echo_netcat(route_status->routeMessageFoward(), DEFAULT_COMM_MSG_PORT);
        PAPI::system::sleepLessThanASecond(0.1);

        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // closeSocket();
    return 0;
}