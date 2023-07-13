#include <ros/ros.h>

#include <wfov_camera_msgs/WFOVImage.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/GPSRTK.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/GPSINPUT.h>

#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <std_msgs/Header.h>
#include <std_msgs/String.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <vector>
#include <string>
#include <map>

#include "/home/pino/pino_ws/papi/PAPI.h"

#define EWOK_TOPIC "/ewok/"

const float min_voltage = 0;
const float max_voltage = 30;
const float min_percentage = 0;
const float max_percentage = 1;

const float GPS_max_horiz_accuracy = 0.1;
const float GPS_max_vert_accuracy = 0.1;
const float GPS_max_HDOP = 1;
const float GPS_max_VDOP = 1;

const uint32_t uint32_zero = 0;
ros::Time time_zero(uint32_zero, uint32_zero);

auto image_fps_duration = ros::Duration(DEFAULT_IMAGE_FPS);

class PeripheralsStatus
{
public:
    ros::NodeHandle nh;
    // ros::NodeHandle nh_private_;

    ros::Subscriber FLIR_sub;
    ros::Subscriber D455_sub;
    ros::Subscriber T265_sub;
    ros::Subscriber TERABEE_sub;
    ros::Subscriber RTK_sub;
    ros::Subscriber FCU_STATE_sub;
    ros::Subscriber FCU_IMU_sub;
    ros::Subscriber FCU_ODOM_sub;
    ros::Subscriber FCU_MAG_sub;
    ros::Subscriber FCU_PRES_sub;
    ros::Subscriber FCU_BAT_sub;
    ros::Subscriber FCU_GPS_sub;

    std::vector<ros::Time> firstTime;
    std::vector<ros::Time> lastTime;

    bool FLIR_image_exist = false;
    bool D455_image_exist = false;
    bool T265_image_exist = false;

    std::vector<int> current_status;
    std::vector<bool> callBack_status;
    std::vector<bool> used;

    std::string mission_id;

    std::string FLIR_image_path;
    std::string D455_image_path;
    std::string T265_image_path;

    std::string MAV_STATE = "";

    ros::Time FLIR_lastImageTime = time_zero;
    ros::Time D455_lastImageTime = time_zero;

    /****************************/

    // Default Constructor
    PeripheralsStatus();

    // Constructor
    PeripheralsStatus(const ros::NodeHandle &_nh);

    // Deconstructor:
    ~PeripheralsStatus();

    // If the callBack function of a subscriber is not be executed, change the current_status of it to NOT_FOUND
    void callBack_exist();

    // Update peripheral usage status
    void addPeripherals(const std::vector<int> &_list);

    // Add mission id to PeripheralsStatus::id
    void addMissionID(const std::string &id);

    // Set the path to camera image file with misison ID
    void setImagePath();

    /****************************/

    void FLIR_CallBack(const wfov_camera_msgs::WFOVImage::ConstPtr &msg);

    void D455_CallBack(const sensor_msgs::Image::ConstPtr &msg);

    void T265_CallBack(const nav_msgs::Odometry::ConstPtr &msg);

    void TERABEE_CallBack(const sensor_msgs::Range::ConstPtr &msg);

    void RTK_CallBack(const mavros_msgs::GPSRTK::ConstPtr &msg);

    void FCU_STATE_CallBack(const mavros_msgs::State::ConstPtr &msg);

    void FCU_IMU_CallBack(const sensor_msgs::Imu::ConstPtr &msg);

    void FCU_ODOM_CallBack(const nav_msgs::Odometry::ConstPtr &msg);

    void FCU_MAG_CallBack(const sensor_msgs::MagneticField::ConstPtr &msg);

    void FCU_PRES_CallBack(const sensor_msgs::FluidPressure::ConstPtr &msg);

    void FCU_BAT_CallBack(const sensor_msgs::BatteryState::ConstPtr &msg);

    void FCU_GPS_CallBack(const mavros_msgs::GPSINPUT::ConstPtr &msg);

    /****************************/

    bool FLIR_exist(const wfov_camera_msgs::WFOVImage::ConstPtr &msg);

    bool D455_exist(const sensor_msgs::Image::ConstPtr &msg);

    bool T265_exist(const nav_msgs::Odometry::ConstPtr &msg);

    bool TERABEE_exist(const sensor_msgs::Range::ConstPtr &msg);

    bool RTK_exist(const mavros_msgs::GPSRTK::ConstPtr &msg);

    bool FCU_STATE_exist(const mavros_msgs::State::ConstPtr &msg);

    bool FCU_IMU_exist(const sensor_msgs::Imu::ConstPtr &msg);

    bool FCU_ODOM_exist(const nav_msgs::Odometry::ConstPtr &msg);

    bool FCU_MAG_exist(const sensor_msgs::MagneticField::ConstPtr &msg);

    bool FCU_PRES_exist(const sensor_msgs::FluidPressure::ConstPtr &msg);

    bool FCU_BAT_exist(const sensor_msgs::BatteryState::ConstPtr &msg);

    bool FCU_GPS_exist(const mavros_msgs::GPSINPUT::ConstPtr &msg);

    /*********************************/

    // Check if it is necessary to send new Image of a device (camera)?
    bool check_sendNewImage(const int _device);

    // Return status of a peripheral with it last and first Time
    int timeStatus(int _peripheral_index);

    // Debug function.
    void debug();

    // Get status in string format divided by '|' character with MAV_STATE at first, then a space
    std::string getStatus_toString();
};

class RouteStatus
{
public:
    ros::NodeHandle nh;
    // ros::NodeHandle nh_private_;

    ros::Subscriber route_sub;

    /****************************/
    // Default Constructor
    RouteStatus();

    // Constructor
    RouteStatus(const ros::NodeHandle &_nh);

    // Deconstructor
    ~RouteStatus();

    /****************************/

    void route_CallBack(const std_msgs::String::ConstPtr &_msg);

    // Return the string in newest_route_msg
    std::string routeMessageFoward();

public:
    std::string newest_route_msg;
};