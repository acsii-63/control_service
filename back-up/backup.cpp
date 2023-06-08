
/***********************************************/

// template <typename message>
// int PeripheralsStatus::device_exist(message _msg, int _dev)
// {
//     int device = static_cast<DEVICE>(_dev); // Device in DEVICE enum list

//     switch (device)
//     {
//     case DEVICE::FLIR:
//         return PeripheralsStatus::FLIR_exist(_msg);
//         break;

//     case DEVICE::D455:
//         return PeripheralsStatus::D455_exist(_msg);
//         break;

//     case DEVICE::changeMe:
//         return PeripheralsStatus::changeMe_exist(_msg);
//         break;

//     case DEVICE::LIDAR:
//         return PERIPHERAL_STATUS::UNSPECIFIED;
//         break;

//     case DEVICE::TERABEE:
//         return PeripheralsStatus::TERABEE_exist(_msg);
//         break;

//     case DEVICE::RTK:
//         return PeripheralsStatus::RTK_exist(_msg);
//         break;

//     case DEVICE::FCU_STATE:
//         return PeripheralsStatus::FCU_STATE_exist(_msg);
//         break;

//     case DEVICE::FCU_IMU:
//         return PeripheralsStatus::FCU_IMU_exist(_msg);
//         break;

//     case DEVICE::FCU_ODOM:
//         return PeripheralsStatus::FCU_ODOM_exist(_msg);
//         break;

//     case DEVICE::FCU_MAG:
//         return PeripheralsStatus::FCU_MAG_exist(_msg);
//         break;

//     case DEVICE::FCU_PRES:
//         return PeripheralsStatus::FCU_PRES_exist(_msg);
//         break;

//     case DEVICE::FCU_BAT:
//         return PeripheralsStatus::FCU_BAT_exist(_msg);
//         break;

//     case DEVICE::FCU_MOTOR:
//         return PERIPHERAL_STATUS::UNSPECIFIED;
//         break;

//     case DEVICE::FCU_AHRS:
//         return PERIPHERAL_STATUS::UNSPECIFIED;
//         break;

//     case DEVICE::FCU_TELE:
//         return PERIPHERAL_STATUS::UNSPECIFIED;
//         break;

//     case DEVICE::FCU_GPS:
//         return PeripheralsStatus::FCU_GPS_exist(_msg);
//         break;

//     default:
//         return INT8_MIN;
//         break;
//     }
// }

callBack_status[DEVICE::changeMe] = true;

if (changeMe_exist(msg))
{
    if (firstTime[DEVICE::changeMe].toSec() <= 0)
        firstTime[DEVICE::changeMe] = ros::Time::now();

    lastTime[DEVICE::changeMe] = ros::Time::now();
}
else
{
    firstTime[DEVICE::changeMe] = time_zero;
}

current_status[DEVICE::changeMe] = timeStatus(DEVICE::changeMe);