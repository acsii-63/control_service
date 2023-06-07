#include "peripherals_status/peripherals_status.h"

#include "/home/pino/pino_ws/papi/PAPI.h"

PeripheralsStatus::PeripheralsStatus()
{
}

PeripheralsStatus::PeripheralsStatus(const ros::NodeHandle &_nh) : nh(_nh)
{
    FLIR_sub = nh.subscribe("/camera/image", 5, PeripheralsStatus::FLIR_CallBack);
    D455_sub = nh.subscribe("/d400/color/image_raw", 5, PeripheralsStatus::D455_CallBack);
    T265_sub = nh.subscribe("/t265/odom/sample", 5, PeripheralsStatus::T265_CallBack);
    TERABEE_sub = nh.subscribe(" /mavros/px4flow/ground_distance", 5, PeripheralsStatus::TERABEE_CallBack);
    RTK_sub = nh.subscribe("/mavros/gpsstatus/gps2/rtk", 5, PeripheralsStatus::RTK_CallBack);
    FCU_STATE_sub = nh.subscribe("/mavros/state", 5, PeripheralsStatus::FCU_STATE_CallBack);
    FCU_IMU_sub = nh.subscribe("/mavros/imu/data_raw", 5, PeripheralsStatus::FCU_IMU_CallBack);
    FCU_ODOM_sub = nh.subscribe("/mavros/odometry/in", 5, PeripheralsStatus::FCU_ODOM_CallBack);
    FCU_MAG_sub = nh.subscribe("/mavros/imu/mag", 5, PeripheralsStatus::FCU_MAG_CallBack);
    FCU_PRES_sub = nh.subscribe("/mavros/imu/static_pressure", 5, PeripheralsStatus::FCU_PRES_CallBack);
    FCU_BAT_sub = nh.subscribe("/mavros/battery", 5, PeripheralsStatus::FCU_BAT_CallBack);
    FCU_GPS_sub = nh.subscribe(" /mavros/gps_input/gps_input ", 5, PeripheralsStatus::FCU_GPS_CallBack);
}

PeripheralsStatus::~PeripheralsStatus()
{
}

void PeripheralsStatus::FLIR_CallBack(const wfov_camera_msgs::WFOVImage::ConstPtr &msg)
{
}

void PeripheralsStatus::D455_CallBack(const sensor_msgs::Image::ConstPtr &msg)
{
}

void PeripheralsStatus::T265_CallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
}

void PeripheralsStatus::TERABEE_CallBack(const sensor_msgs::Range::ConstPtr &msg)
{
}

void PeripheralsStatus::RTK_CallBack(const mavros_msgs::GPSRTK::ConstPtr &msg)
{
}

void PeripheralsStatus::FCU_STATE_CallBack(const mavros_msgs::State::ConstPtr &msg)
{
}

void PeripheralsStatus::FCU_IMU_CallBack(const sensor_msgs::Imu::ConstPtr &msg)
{
}

void PeripheralsStatus::FCU_ODOM_CallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
}

void PeripheralsStatus::FCU_MAG_CallBack(const sensor_msgs::MagneticField::ConstPtr &msg)
{
}

void PeripheralsStatus::FCU_PRES_CallBack(const sensor_msgs::FluidPressure::ConstPtr &msg)
{
}

void PeripheralsStatus::FCU_BAT_CallBack(const sensor_msgs::BatteryState::ConstPtr &msg)
{
}

void PeripheralsStatus::FCU_GPS_CallBack(const mavros_msgs::GPSINPUT::ConstPtr &msg)
{
}

/***********************************************/

template <typename message>
int PeripheralsStatus::device_exist(message _msg, int _dev)
{
    int device = static_cast<DEVICE>(_dev); // Device in DEVICE enum list

    switch (device)
    {
    case DEVICE::FLIR:
        return PeripheralsStatus::FLIR_exist(_msg);
        break;

    case DEVICE::D455:
        return PeripheralsStatus::D455_exist(_msg);
        break;

    case DEVICE::T265:
        return PeripheralsStatus::T265_exist(_msg);
        break;

    case DEVICE::LIDAR:
        return PERIPHERAL_STATUS::UNSPECIFIED;
        break;

    case DEVICE::TERABEE:
        return PeripheralsStatus::TERABEE_exist(_msg);
        break;

    case DEVICE::RTK:
        return PeripheralsStatus::RTK_exist(_msg);
        break;

    case DEVICE::FCU_STATE:
        return PeripheralsStatus::FCU_STATE_exist(_msg);
        break;

    case DEVICE::FCU_IMU:
        return PeripheralsStatus::FCU_IMU_exist(_msg);
        break;

    case DEVICE::FCU_ODOM:
        return PeripheralsStatus::FCU_ODOM_exist(_msg);
        break;

    case DEVICE::FCU_MAG:
        return PeripheralsStatus::FCU_MAG_exist(_msg);
        break;

    case DEVICE::FCU_PRES:
        return PeripheralsStatus::FCU_PRES_exist(_msg);
        break;

    case DEVICE::FCU_BAT:
        return PeripheralsStatus::FCU_BAT_exist(_msg);
        break;

    case DEVICE::FCU_MOTOR:
        return PERIPHERAL_STATUS::UNSPECIFIED;
        break;

    case DEVICE::FCU_AHRS:
        return PERIPHERAL_STATUS::UNSPECIFIED;
        break;

    case DEVICE::FCU_TELE:
        return PERIPHERAL_STATUS::UNSPECIFIED;
        break;

    case DEVICE::FCU_GPS:
        return PeripheralsStatus::FCU_GPS_exist(_msg);
        break;

    default:
        return INT8_MIN;
        break;
    }
}

int PeripheralsStatus::timeStatus(const int _peripheral_index)
{
    ros::Duration timeSinceLastImage = ros::Time::now() - PeripheralsStatus::lastTime[_peripheral_index];
    if (timeSinceLastImage.toSec() > 1.0)
        return PERIPHERAL_STATUS::INACTIVE;

    ros::Duration timeSinceFirstImage = ros::Time::now() - PeripheralsStatus::firstTime[_peripheral_index];
    if (timeSinceFirstImage.toSec() > 5.0)
        return PERIPHERAL_STATUS::ACTIVE;
    else
        return PERIPHERAL_STATUS::WAITING_FOR_ACTIVE;
}

int PeripheralsStatus::FLIR_exist(const wfov_camera_msgs::WFOVImage::ConstPtr &msg)
{
    const sensor_msgs::Image &ros_image = msg->image;
}

int PeripheralsStatus::D455_exist(const sensor_msgs::Image::ConstPtr &msg)
{
}

int PeripheralsStatus::T265_exist(const nav_msgs::Odometry::ConstPtr &msg)
{
}

int PeripheralsStatus::TERABEE_exist(const sensor_msgs::Range::ConstPtr &msg)
{
}

int PeripheralsStatus::RTK_exist(const mavros_msgs::GPSRTK::ConstPtr &msg)
{
}

int PeripheralsStatus::FCU_STATE_exist(const mavros_msgs::State::ConstPtr &msg)
{
}

int PeripheralsStatus::FCU_IMU_exist(const sensor_msgs::Imu::ConstPtr &msg)
{
}

int PeripheralsStatus::FCU_ODOM_exist(const nav_msgs::Odometry::ConstPtr &msg)
{
}

int PeripheralsStatus::FCU_MAG_exist(const sensor_msgs::MagneticField::ConstPtr &msg)
{
}

int PeripheralsStatus::FCU_PRES_exist(const sensor_msgs::FluidPressure::ConstPtr &msg)
{
}

int PeripheralsStatus::FCU_BAT_exist(const sensor_msgs::BatteryState::ConstPtr &msg)
{
}

int PeripheralsStatus::FCU_GPS_exist(const mavros_msgs::GPSINPUT::ConstPtr &msg)
{
}