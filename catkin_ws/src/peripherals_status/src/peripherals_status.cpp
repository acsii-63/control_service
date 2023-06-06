#include "peripherals_status/peripherals_status.h"

#include "/home/pino/pino_ws/papi/PAPI.h"

template <typename message>
int device_exist(message _msg, int _dev)
{
    int device = static_cast<DEVICE>(_dev); // Device in DEVICE enum list

    switch (device)
    {
    case DEVICE::FLIR:

        break;

    case DEVICE::D455:

        break;

    case DEVICE::T265:

        break;

    case DEVICE::LIDAR:

        break;

    case DEVICE::TERABEE:

        break;

    case DEVICE::RTK:

        break;

    case DEVICE::FCU_STATE:

        break;

    case DEVICE::FCU_IMU:

        break;

    case DEVICE::FCU_ODOM:

        break;

    case DEVICE::FCU_MAG:

        break;

    case DEVICE::FCU_PRES:

        break;

    case DEVICE::FCU_BAT:

        break;

    case DEVICE::FCU_MOTOR:

        break;

    case DEVICE::FCU_AHRS:

        break;

    case DEVICE::FCU_TELE:

        break;

    case DEVICE::FCU_GPS:

        break;

    default:

        break;
    }
}

int FLIR_exist(const wfov_camera_msgs::WFOVImage::ConstPtr &msg)
{
}

int D455_exist(const sensor_msgs::Image::ConstPtr &msg)
{
}

int T265_exist(const nav_msgs::Odometry::ConstPtr &msg)
{
}