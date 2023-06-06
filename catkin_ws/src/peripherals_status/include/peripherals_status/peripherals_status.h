#include <ros/ros.h>

#include <wfov_camera_msgs/WFOVImage.h>

#include <sensor_msgs/Image.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <vector>
#include <string>
#include <map>

enum DEVICE : int
{
    FLIR = 0,  // FLIR
    D455,      // D455
    T265,      // T265
    LIDAR,     // LIDAR
    TERABEE,   // Range Finder
    RTK,       // RTK
    FCU_STATE, // MAV State
    FCU_IMU,   // MAV IMU
    FCU_ODOM,  // MAV Odometry
    FCU_MAG,   // MAV Magnetometer
    FCU_PRES,  // MAV Absolute Pressure
    FCU_BAT,   // MAV Battery
    FCU_MOTOR, // MAV Motor outputs, control
    FCU_AHRS,  // MAV Accelerometer
    FCU_TELE,  // MAV Telemetry (P900?)
    FCU_GPS    // MAV GPS
};

std::vector<ros::Time> firstTime;
std::vector<ros::Time> lastTime;

int FLIR_exist(const wfov_camera_msgs::WFOVImage::ConstPtr &msg);

int D455_exist(const sensor_msgs::Image::ConstPtr &msg);

int T265_exist(const nav_msgs::Odometry::ConstPtr &msg);

// int TERABEE_exist()


// Device exist?
template <typename message>
int device_exist(message _msg, int _dev);