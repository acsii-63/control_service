#include "peripherals_status/peripherals_status.h"
#include "/home/pino/pino_ws/papi/PAPI.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "automatic");
    ros::NodeHandle nh("");
    PeripheralsStatus *peripherals_status = new PeripheralsStatus(nh);

    std::vector<int> list{DEVICE::FLIR, DEVICE::D455, DEVICE::T265};
    peripherals_status->addPeripherals(list);

    while (ros::ok())
    {
        peripherals_status->debug();

        peripherals_status->callBack_exist();

        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    return 0;
}