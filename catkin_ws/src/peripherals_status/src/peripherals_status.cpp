#include "peripherals_status/peripherals_status.h"

#include "/home/pino/pino_ws/papi/PAPI.h"

PeripheralsStatus::PeripheralsStatus()
{
}

PeripheralsStatus::PeripheralsStatus(const ros::NodeHandle &_nh) : nh(_nh)
{
    FLIR_sub = nh.subscribe("/camera/image", 5, &PeripheralsStatus::FLIR_CallBack, this);
    D455_sub = nh.subscribe("/d400/color/image_raw", 5, &PeripheralsStatus::D455_CallBack, this);
    T265_sub = nh.subscribe("/t265/odom/sample", 5, &PeripheralsStatus::T265_CallBack, this);
    TERABEE_sub = nh.subscribe("/mavros/px4flow/ground_distance", 5, &PeripheralsStatus::TERABEE_CallBack, this);
    RTK_sub = nh.subscribe("/mavros/gpsstatus/gps2/rtk", 5, &PeripheralsStatus::RTK_CallBack, this);
    FCU_STATE_sub = nh.subscribe("/mavros/state", 5, &PeripheralsStatus::FCU_STATE_CallBack, this);
    FCU_IMU_sub = nh.subscribe("/mavros/imu/data_raw", 5, &PeripheralsStatus::FCU_IMU_CallBack, this);
    FCU_ODOM_sub = nh.subscribe("/mavros/odometry/in", 5, &PeripheralsStatus::FCU_ODOM_CallBack, this);
    FCU_MAG_sub = nh.subscribe("/mavros/imu/mag", 5, &PeripheralsStatus::FCU_MAG_CallBack, this);
    FCU_PRES_sub = nh.subscribe("/mavros/imu/static_pressure", 5, &PeripheralsStatus::FCU_PRES_CallBack, this);
    FCU_BAT_sub = nh.subscribe("/mavros/battery", 5, &PeripheralsStatus::FCU_BAT_CallBack, this);
    FCU_GPS_sub = nh.subscribe("/mavros/gps_input/gps_input", 5, &PeripheralsStatus::FCU_GPS_CallBack, this);

    firstTime.reserve(20);
    lastTime.reserve(20);
    current_status.reserve(20);
    callBack_status.reserve(20);
    used.reserve(20);

    for (int i = 0; i < 16; i++)
    {
        firstTime[i] = time_zero;
        current_status[i] = PERIPHERAL_STATUS::UNSPECIFIED;
        callBack_status[i] = false;
        used[i] = false;
    }

    FLIR_image_path = "/home/pino/image/flir_image.png";
    D455_image_path = "/home/pino/image/d455_image.png";
    T265_image_path = "/home/pino/image/t265_image.png";
}

PeripheralsStatus::~PeripheralsStatus()
{
}

void PeripheralsStatus::FLIR_CallBack(const wfov_camera_msgs::WFOVImage::ConstPtr &msg)
{
    callBack_status[DEVICE::FLIR] = true;

    const sensor_msgs::Image &ros_image = msg->image;

    if (!FLIR_image_exist)
    {
        FLIR_image_exist = true;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat cv_image = cv_ptr->image;
        if (!cv::imwrite(FLIR_image_path, cv_image))
            ROS_ERROR("Can not write FLIR image to: %s", FLIR_image_path.c_str());
    }

    if (FLIR_exist(msg))
    {
        // First image
        if (firstTime[DEVICE::FLIR].toSec() <= 0)
            firstTime[DEVICE::FLIR] = ros::Time::now();

        lastTime[DEVICE::FLIR] = ros::Time::now();
    }
    else
    {
        firstTime[DEVICE::FLIR] = time_zero;
    }

    current_status[DEVICE::FLIR] = timeStatus(DEVICE::FLIR);

    for (int i = 0; i <= 15; i++)
        std::cout << current_status[i] << " ";
    std::cout << std::endl;
    std::cout << "==========================" << std::endl;
}

void PeripheralsStatus::D455_CallBack(const sensor_msgs::Image::ConstPtr &msg)
{
    callBack_status[DEVICE::D455] = true;

    const sensor_msgs::Image &ros_image = *msg;

    if (!D455_image_exist)
    {
        D455_image_exist = true;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat cv_image = cv_ptr->image;
        if (!cv::imwrite(D455_image_path, cv_image))
            ROS_ERROR("Can not write D455 image to: %s", D455_image_path.c_str());
    }

    // if (ros_image.data.size() > 0)
    if (D455_exist(msg))
    {
        // First image
        if (firstTime[DEVICE::D455].toSec() <= 0)
            firstTime[DEVICE::D455] = ros::Time::now();

        lastTime[DEVICE::D455] = ros::Time::now();
    }
    else
    {
        firstTime[DEVICE::D455] = time_zero;
    }

    current_status[DEVICE::D455] = timeStatus(DEVICE::D455);
}

void PeripheralsStatus::T265_CallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    callBack_status[DEVICE::T265] = true;

    if (T265_exist(msg))
    {
        if (firstTime[DEVICE::T265].toSec() <= 0)
            firstTime[DEVICE::T265] = ros::Time::now();

        lastTime[DEVICE::T265] = ros::Time::now();
    }
    else
    {
        firstTime[DEVICE::T265] = time_zero;
    }

    current_status[DEVICE::T265] = timeStatus(DEVICE::T265);
}

void PeripheralsStatus::TERABEE_CallBack(const sensor_msgs::Range::ConstPtr &msg)
{
    callBack_status[DEVICE::TERABEE] = true;

    if (TERABEE_exist(msg))
    {
        if (firstTime[DEVICE::TERABEE].toSec() <= 0)
            firstTime[DEVICE::TERABEE] = ros::Time::now();

        lastTime[DEVICE::TERABEE] = ros::Time::now();
    }
    else
    {
        firstTime[DEVICE::TERABEE] = time_zero;
    }

    current_status[DEVICE::TERABEE] = timeStatus(DEVICE::TERABEE);
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

int PeripheralsStatus::timeStatus(int _peripheral_index)
{
    ros::Duration timeSinceLastImage = ros::Time::now() - lastTime[_peripheral_index];
    if (timeSinceLastImage.toSec() > 1.0)
        return PERIPHERAL_STATUS::INACTIVE;

    ros::Duration timeSinceFirstImage = ros::Time::now() - firstTime[_peripheral_index];
    if (timeSinceFirstImage.toSec() > 5.0)
        return PERIPHERAL_STATUS::ACTIVE;
    else
        return PERIPHERAL_STATUS::WAITING_FOR_ACTIVE;
}

bool PeripheralsStatus::FLIR_exist(const wfov_camera_msgs::WFOVImage::ConstPtr &msg)
{
    const sensor_msgs::Image &ros_image = msg->image;
    return (ros_image.data.size() > 0) ? true : false;
}

bool PeripheralsStatus::D455_exist(const sensor_msgs::Image::ConstPtr &msg)
{
    const sensor_msgs::Image &ros_image = *msg;
    return (ros_image.data.size() > 0) ? true : false;
}

bool PeripheralsStatus::T265_exist(const nav_msgs::Odometry::ConstPtr &msg)
{
    const geometry_msgs::PoseWithCovariance &pose_with_convariance = msg->pose;
    const geometry_msgs::Point &point = msg->pose.pose.position;

    if (point.x != 0 && point.y != 0 && point.z != 0)
        return true;
    return false;
}

bool PeripheralsStatus::TERABEE_exist(const sensor_msgs::Range::ConstPtr &msg)
{
    const sensor_msgs::Range &altitude = *msg;
    if (msg->range >= msg->min_range && msg->range <= msg->max_range)
        return true;
    return false;
}

bool PeripheralsStatus::RTK_exist(const mavros_msgs::GPSRTK::ConstPtr &msg)
{
    callBack_status[DEVICE::RTK] = true;

    if (RTK_exist(msg))
    {
        if (firstTime[DEVICE::RTK].toSec() <= 0)
            firstTime[DEVICE::RTK] = ros::Time::now();

        lastTime[DEVICE::RTK] = ros::Time::now();
    }
    else
    {
        firstTime[DEVICE::RTK] = time_zero;
    }

    current_status[DEVICE::RTK] = timeStatus(DEVICE::RTK);
}

bool PeripheralsStatus::FCU_STATE_exist(const mavros_msgs::State::ConstPtr &msg)
{
    callBack_status[DEVICE::FCU_STATE] = true;

    if (FCU_STATE_exist(msg))
    {
        if (firstTime[DEVICE::FCU_STATE].toSec() <= 0)
            firstTime[DEVICE::FCU_STATE] = ros::Time::now();

        lastTime[DEVICE::FCU_STATE] = ros::Time::now();
    }
    else
    {
        firstTime[DEVICE::FCU_STATE] = time_zero;
    }

    current_status[DEVICE::FCU_STATE] = timeStatus(DEVICE::FCU_STATE);
}

bool PeripheralsStatus::FCU_IMU_exist(const sensor_msgs::Imu::ConstPtr &msg)
{
    callBack_status[DEVICE::FCU_IMU] = true;

    if (FCU_IMU_exist(msg))
    {
        if (firstTime[DEVICE::FCU_IMU].toSec() <= 0)
            firstTime[DEVICE::FCU_IMU] = ros::Time::now();

        lastTime[DEVICE::FCU_IMU] = ros::Time::now();
    }
    else
    {
        firstTime[DEVICE::FCU_IMU] = time_zero;
    }

    current_status[DEVICE::FCU_IMU] = timeStatus(DEVICE::FCU_IMU);
}

bool PeripheralsStatus::FCU_ODOM_exist(const nav_msgs::Odometry::ConstPtr &msg)
{
    callBack_status[DEVICE::FCU_ODOM] = true;

    if (FCU_ODOM_exist(msg))
    {
        if (firstTime[DEVICE::FCU_ODOM].toSec() <= 0)
            firstTime[DEVICE::FCU_ODOM] = ros::Time::now();

        lastTime[DEVICE::FCU_ODOM] = ros::Time::now();
    }
    else
    {
        firstTime[DEVICE::FCU_ODOM] = time_zero;
    }

    current_status[DEVICE::FCU_ODOM] = timeStatus(DEVICE::FCU_ODOM);
}

bool PeripheralsStatus::FCU_MAG_exist(const sensor_msgs::MagneticField::ConstPtr &msg)
{
    callBack_status[DEVICE::FCU_MAG] = true;

    if (FCU_MAG_exist(msg))
    {
        if (firstTime[DEVICE::FCU_MAG].toSec() <= 0)
            firstTime[DEVICE::FCU_MAG] = ros::Time::now();

        lastTime[DEVICE::FCU_MAG] = ros::Time::now();
    }
    else
    {
        firstTime[DEVICE::FCU_MAG] = time_zero;
    }

    current_status[DEVICE::FCU_MAG] = timeStatus(DEVICE::FCU_MAG);
}

bool PeripheralsStatus::FCU_PRES_exist(const sensor_msgs::FluidPressure::ConstPtr &msg)
{
    callBack_status[DEVICE::FCU_PRES] = true;

    if (FCU_PRES_exist(msg))
    {
        if (firstTime[DEVICE::FCU_PRES].toSec() <= 0)
            firstTime[DEVICE::FCU_PRES] = ros::Time::now();

        lastTime[DEVICE::FCU_PRES] = ros::Time::now();
    }
    else
    {
        firstTime[DEVICE::FCU_PRES] = time_zero;
    }

    current_status[DEVICE::FCU_PRES] = timeStatus(DEVICE::FCU_PRES);
}

bool PeripheralsStatus::FCU_BAT_exist(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    callBack_status[DEVICE::FCU_BAT] = true;

    if (FCU_BAT_exist(msg))
    {
        if (firstTime[DEVICE::FCU_BAT].toSec() <= 0)
            firstTime[DEVICE::FCU_BAT] = ros::Time::now();

        lastTime[DEVICE::FCU_BAT] = ros::Time::now();
    }
    else
    {
        firstTime[DEVICE::FCU_BAT] = time_zero;
    }

    current_status[DEVICE::FCU_BAT] = timeStatus(DEVICE::FCU_BAT);
}

bool PeripheralsStatus::FCU_GPS_exist(const mavros_msgs::GPSINPUT::ConstPtr &msg)
{
    callBack_status[DEVICE::FCU_GPS] = true;

    if (FCU_GPS_exist(msg))
    {
        if (firstTime[DEVICE::FCU_GPS].toSec() <= 0)
            firstTime[DEVICE::FCU_GPS] = ros::Time::now();

        lastTime[DEVICE::FCU_GPS] = ros::Time::now();
    }
    else
    {
        firstTime[DEVICE::FCU_GPS] = time_zero;
    }

    current_status[DEVICE::FCU_GPS] = timeStatus(DEVICE::FCU_GPS);
}

void PeripheralsStatus::callBack_exist()
{
    for (int i = 0; i < 16; i++)
    {
        if (!callBack_status[i] && used[i])
            current_status[i] = PERIPHERAL_STATUS::NOT_FOUND;
    }
}

void PeripheralsStatus::addPeripherals(const std::vector<int> &_list)
{
    std::vector<int> list = _list;
    for (int i = 0; i < list.size(); i++)
        used[list[i]] = true;
}

void PeripheralsStatus::debug()
{
    std::cout << "USED? : ";
    for (int i = 0; i <= 15; i++)
        std::cout << std::setw(2) << used[i] << " ";
    std::cout << std::endl;

    std::cout << "STATUS: ";
    for (int i = 0; i <= 15; i++)
        std::cout << std::setw(2) << current_status[i] << " ";
    std::cout << std::endl;

    std::cout << "==========================" << std::endl;
}