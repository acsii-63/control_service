#include "peripherals_status/peripherals_status.h"
#include "/home/pino/pino_ws/papi/PAPI.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "automatic");
    ros::NodeHandle nh("");
    PeripheralsStatus *peripherals_status = new PeripheralsStatus(nh);

    ros::spin();

    return 0;
}