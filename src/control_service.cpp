/* Compile:
    g++ control_service.cpp -o control_service -pthread -ljsoncpp -lyaml-cpp
 */
#include "/home/pino/pino_ws/papi/PAPI.h"

MissionRequest mission;

PAPI::communication::Server server_peripherals(DEFAULT_STATUS_CONTROL_SERVICE_PORT);
PAPI::communication::Client client_peripherals(LOCAL_HOST, DEFAULT_STATUS_ROS_NODE_PORT);

PAPI::communication::Server server_route(DEFAULT_ROUTE_CONTROL_SERVICE_PORT);
PAPI::communication::Client client_route(LOCAL_HOST, DEFAULT_ROUTE_ROS_NODE_PORT);

void driver_loader()
{
    std::string cmd;               // command
    std::vector<std::string> argv; // arguments vector
    // Spinnaker (FLIR)
    cmd = "roslaunch";
    argv.push_back("spinnaker_camera_driver");
    argv.push_back("color_cam.launch");
    argv.push_back(">> /home/pino/logs/roslaunch_logs/spinnaker.log");
    argv.push_back("2>&1 &");

    PAPI::system::runCommand_system(cmd, argv);
    sleep(1);
    cmd.clear();
    argv.clear();

    // Realsense2 (D455 and T265)
    cmd = "roslaunch";
    argv.push_back("realsense2_camera");
    argv.push_back("rs_d400_and_t265.launch");
    argv.push_back(">> /home/pino/logs/roslaunch_logs/realsense.log");
    argv.push_back("2>&1 &");

    PAPI::system::runCommand_system(cmd, argv);
    sleep(1);
    cmd.clear();
    argv.clear();
}

// Send Image, Status and get Response
bool initCheck()
{
    std::string first_msg = "";
    // Record the starting time
    auto startTime_firstStatus = std::chrono::high_resolution_clock::now();
    PAPI::communication::sendMessage_echo_netcat("[ INFO] Waiting for first status message...", DEFAULT_COMM_MSG_PORT);
    do
    {
        first_msg = client_peripherals.receiveMessage();
        startTime_firstStatus = std::chrono::high_resolution_clock::now();
    } while (first_msg.empty());

    // Set the timeout duration
    auto wait_for_active_duration = std::chrono::seconds(DEFAULT_TIME_WAIT_FOR_ACTIVE);
    std::chrono::seconds elapsed_duration;
    std::string last_msg = client_peripherals.receiveMessage();

    PAPI::communication::sendMessage_echo_netcat("[ INFO] Waiting for final status result...", DEFAULT_COMM_MSG_PORT);
    do
    {
        // Check if the timeout has occurred
        auto currentTime = std::chrono::high_resolution_clock::now();
        elapsed_duration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime_firstStatus);

        last_msg = client_peripherals.receiveMessage();

    } while (elapsed_duration < wait_for_active_duration);

    std::cout << "Last status message: " << last_msg << std::endl;

    std::string mav_state;
    std::vector<int> peripherals_status_vector = PAPI::system::getPeripheralsStatus_fromString_toVector(last_msg, mav_state);

    std::vector<int> peripherals_list;
    mission.sequence_istructions[0]->Init_getPeripherals(peripherals_list);

    std::cout << "PASS #1.\n"; /************************************/

    int num_of_images = 0;

    auto it = std::find(peripherals_list.begin(), peripherals_list.end(), Peripheral::PERIPHERAL_CAM_DOWNWARD);
    if (it != peripherals_list.end() && peripherals_status_vector[DEVICE::FLIR] == PERIPHERAL_STATUS::ACTIVE)
    {
        PAPI::system::sendImage(Peripheral::PERIPHERAL_CAM_DOWNWARD, mission.id);
        PAPI::communication::sendMessage_echo_netcat("[ INFO] FLIR image sended.", DEFAULT_COMM_MSG_PORT);
        ++num_of_images;
    }
    else
        PAPI::communication::sendMessage_echo_netcat("[ERROR] FLIR camera is not activated and will not send the image to GCS.", DEFAULT_COMM_MSG_PORT);
    PAPI::system::sleepLessThanASecond(0.1);

    it = std::find(peripherals_list.begin(), peripherals_list.end(), Peripheral::PERIPHERAL_CAM_FORWARD);
    if (it != peripherals_list.end() && peripherals_status_vector[DEVICE::D455] == PERIPHERAL_STATUS::ACTIVE)
    {
        PAPI::system::sendImage(Peripheral::PERIPHERAL_CAM_FORWARD, mission.id);
        PAPI::communication::sendMessage_echo_netcat("[ INFO] D455 image sended.", DEFAULT_COMM_MSG_PORT);
        ++num_of_images;
    }
    else
        PAPI::communication::sendMessage_echo_netcat("[ERROR] D455 camera is not activated and will not send the image to GCS.", DEFAULT_COMM_MSG_PORT);
    PAPI::system::sleepLessThanASecond(0.1);

    PAPI::communication::sendMessage_echo_netcat(PAPI::system::statusParsing(last_msg), DEFAULT_COMM_MSG_PORT);
    PAPI::system::sleepLessThanASecond(0.1);

    std::cout << "PASS #2.\n"; /************************************/

    std::string confirm_msg = "";                                                              // Confirm message
    auto wait_for_image_confirm_timeout = std::chrono::seconds(DEFAULT_IMAGE_CONFIRM_TIMEOUT); // Set the timeout duration
    auto sendTime = std::chrono::high_resolution_clock::now();                                 // Time after send image(s)
    std::vector<std::string> flags;                                                            // Vector contain FLAGs
    PAPI::communication::sendMessage_echo_netcat("[ INFO] Waiting for Image confirm from GCS.", DEFAULT_COMM_MSG_PORT);
    PAPI::system::sleepLessThanASecond(0.1);

    if (num_of_images > 0)
        do
        { // Check if the timeout has occurred
            auto currentTime = std::chrono::high_resolution_clock::now();
            elapsed_duration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - sendTime);

            std::string current_flag = PAPI::communication::receiveMessage_netcat(DEFAULT_CONTROL_CONFIRM_PORT, DEFAULT_IMAGE_CONFIRM_TIMEOUT);
            std::cout << current_flag << std::endl;
            flags.push_back(current_flag);
        } while (elapsed_duration < wait_for_image_confirm_timeout && flags.size() < num_of_images);

    std::cout << "PASS #3.\n"; /************************************/

    if (num_of_images > 0)
    {
        if (flags.size() < num_of_images)
        {
            PAPI::communication::sendMessage_echo_netcat("[ERROR] Stop Init: Missing FLAG(s) after TIMEOUT duration.", DEFAULT_COMM_MSG_PORT);
            PAPI::system::sleepLessThanASecond(0.1);
            return false;
        }
        else
        {
            PAPI::communication::sendMessage_echo_netcat("[ INFO] Got enough Image confirmation from GCS.", DEFAULT_COMM_MSG_PORT);
            PAPI::system::sleepLessThanASecond(0.1);
            // flags = PAPI::system::readAllFLAGsFromFile(DEFAULT_MESSAGE_FILE_PATH);
        }

        if (!PAPI::system::checkAllFLAG(flags))
        {
            PAPI::communication::sendMessage_echo_netcat("[ERROR] Stop Init: Reject from GCS (Camera).", DEFAULT_COMM_MSG_PORT);
            PAPI::system::sleepLessThanASecond(0.1);
            return false;
        }
        else
        {
            PAPI::communication::sendMessage_echo_netcat("[ INFO] Allowed from GCS (Camera).", DEFAULT_COMM_MSG_PORT);
            PAPI::system::sleepLessThanASecond(0.1);
        }
    }

    // if (!PAPI::drone::peripheralsCheck(peripherals_status_vector))
    // {
    //     PAPI::communication::sendMessage_echo_netcat("[ERROR] Stop Init: Peripherals check failed.", DEFAULT_COMM_MSG_PORT);
    //     PAPI::system::sleepLessThanASecond(0.1);
    //     return false;
    // }
    // else
    // {
    //     PAPI::communication::sendMessage_echo_netcat("[ INFO] Peripherals check passed. Everything is working.", DEFAULT_COMM_MSG_PORT);
    //     PAPI::system::sleepLessThanASecond(0.1);
    // }

    std::cout << "PASS #4.\n"; /************************************/

    PAPI::communication::sendMessage_echo_netcat("[ INFO] Waiting for permission to fly. Timeout: 120 seconds.", DEFAULT_COMM_MSG_PORT);
    PAPI::system::sleepLessThanASecond(0.1);
    std::string permission;

    if (!PAPI::system::getNewestFLAG(DEFAULT_CONTROL_CONFIRM_PORT, permission, DEFAULT_GCS_CONFIRM_TIMEOUT))
    {
        PAPI::communication::sendMessage_echo_netcat("[ERROR] UAV will not be airborne or engaged in flight activities.", DEFAULT_COMM_MSG_PORT);
        PAPI::system::sleepLessThanASecond(0.1);
        return false;
    }
    else
    {
        if (permission == FLAG_DENY_TO_FLY)
        {
            PAPI::communication::sendMessage_echo_netcat("[ERROR] UAV will not be airborne or engaged in flight activities: GCS denied.", DEFAULT_COMM_MSG_PORT);
            PAPI::system::sleepLessThanASecond(0.1);
            return false;
        }
        else if (permission == FLAG_ALLOW_TO_FLY)
        {
            PAPI::communication::sendMessage_echo_netcat("[ INFO] UAV is going to execute the flight mission: GCS allowed.", DEFAULT_COMM_MSG_PORT);
            PAPI::system::sleepLessThanASecond(0.1);
        }
    }

    return true;
}

bool InitSequenceAddition()
{
    /* Connect to control node: */
    std::cout << "START CONNECTING." << std::endl;
    int client_connection_result = client_peripherals.clientStart();
    PAPI::system::sleepLessThanASecond(0.5);
    int server_connection_result = server_peripherals.serverStart();
    if (client_connection_result == -1 || server_connection_result == -1)
    {
        PAPI::communication::sendMessage_echo_netcat("[ WARN] Cannot connect to control node.", DEFAULT_COMM_MSG_PORT);
        PAPI::system::sleepLessThanASecond(0.1);
        return false;
    }

    /************************************/

    /* Peripherals usage status change: */
    std::vector<int> list;
    mission.sequence_istructions[0]->Init_getPeripherals(list);
    std::stringstream ss;
    for (auto i = 0; i < list.size(); i++)
        ss << list[i] << "|";
    server_peripherals.sendMsg(ss.str());
    PAPI::system::sleepLessThanASecond(0.5);
    std::cout << ss.str() << std::endl;

    /* Send mission id: */
    server_peripherals.sendMsg(mission.id);
    std::cout << "id: " << mission.id << std::endl;
    PAPI::system::sleepLessThanASecond(0.5);

    /************************************/

    /* Send Image, Status and get Response: */
    return (initCheck());
}

bool TravelSequenceAddition()
{
    // /* Connect to route status node: */
    // int client_connection_result = client_route.clientStart();
    // int server_connection_result = server_route.serverStart();
    // if (client_connection_result == -1 || server_connection_result == -1)
    // {
    //     PAPI::communication::sendMessage_echo_netcat("[ WARN] Cannot connect to route status node.", DEFAULT_COMM_MSG_PORT);
    //     PAPI::system::sleepLessThanASecond(0.1);
    // }

    /* Run control_pkg/route for route notifications: */
    std::string route_cmd = "rosrun";
    std::vector<std::string> route_argv;
    route_argv.push_back("control_pkg");
    route_argv.push_back("route");
    route_argv.push_back("> /home/pino/logs/rosrun_logs/route_log.log");
    route_argv.push_back("2>&1 &");

    PAPI::system::runCommand_system(route_cmd, route_argv);

    /*  */

    return true;
}

bool ActionSequenceAddition()
{

    return true;
}

bool TravelInstructionCleaner(const int &_index)
{
    const int planner = mission.sequence_istructions[_index]->Travel_getPlanner();

    std::vector<std::string> node_list;
    std::vector<std::string> ewok_node_list = {"offboard_node", "spline_optimization_example", "rviz"};

    switch (planner)
    {
    case Planner::PLANNER_EGO:
        node_list = ewok_node_list; // For testing with ewok
        break;

    case Planner::PLANNER_FAST:
        node_list = ewok_node_list; // For testing with ewok
        break;

    case Planner::PLANNER_MARKER:
        node_list = ewok_node_list; // For testing with ewok
        break;

    case Planner::PLANNER_SAFELAND:
        node_list = ewok_node_list; // For testing with ewok
        break;

    default:
        std::cerr << "Planner unspecified.\n";
        return false;
    }

    std::vector<std::string> temp_vector;
    std::vector<std::string> pid_list;
    for (const auto &node : node_list)
    {
        temp_vector = PAPI::system::getPIDList(node);
        pid_list.reserve(pid_list.size() + temp_vector.size());
        pid_list.insert(pid_list.end(), temp_vector.begin(), temp_vector.end());
        temp_vector.clear();
    }

    std::string command = "";
    for (const auto &pid : pid_list)
    {
        command = "kill -9 " + pid;
        std::system(command.c_str());
        PAPI::system::sleepLessThanASecond(0.1);
        command.clear();

        std::cout << pid << " ";
    }
    std::cout << std::endl;

    return true;
}

bool ActionInstructionCleaner(const int &_index)
{
    const int action = mission.sequence_istructions[_index]->Action_getAction();
    switch (action)
    {
    case Action::ACTION_AUTOLAND:
        break;

    case Action::ACTION_DISARM:
        break;

    case Action::ACTION_RELEASE:
        break;

    case Action::ACTION_RTLHOME:
        break;

    case Action::ACTION_SELFCHECK:
        break;

    case Action::ACTION_TAKEOFF:
        break;

    default:
        std::cerr << "Action unspecified.\n";
        return false;
    }

    return true;
}

// [DEMO] Init the system, contain PX4, MAVROS and GEOMETRIC_CONTROLLER.
bool demo()
{
    std::string px4_cmd = "roslaunch";
    std::vector<std::string> px4_argv;
    px4_argv.push_back("px4");
    px4_argv.push_back("posix_sitl.launch");
    px4_argv.push_back("> /home/pino/logs/roslaunch_logs/px4_log.log");
    // px4_argv.push_back("2>&1 &");
    px4_argv.push_back("&");

    std::string mavros_cmd = "roslaunch";
    std::vector<std::string> mavros_argv;
    mavros_argv.push_back("mavros");
    mavros_argv.push_back("px4.launch");
    mavros_argv.push_back("fcu_url:=\"udp://:14540@localhost:14557\"");
    mavros_argv.push_back("> /home/pino/logs/roslaunch_logs/mavros_log.log");
    // mavros_argv.push_back("2>&1 &");
    mavros_argv.push_back("&");

    std::string controller_cmd = "roslaunch";
    std::vector<std::string> controller_argv;
    controller_argv.push_back("geometric_controller");
    controller_argv.push_back("automatic.launch");
    controller_argv.push_back("> /home/pino/logs/roslaunch_logs/controller_log.log");
    // controller_argv.push_back("2>&1 &");
    controller_argv.push_back("&");

    std::string control_ros_status_cmd = "rosrun";
    std::vector<std::string> control_ros_status_argv;
    control_ros_status_argv.push_back("control_pkg");
    control_ros_status_argv.push_back("automatic");
    control_ros_status_argv.push_back("> /home/pino/logs/rosrun_logs/control_log.log");
    // control_ros_status_argv.push_back("peripherals_status");
    // control_ros_status_argv.push_back("> /home/pino/logs/rosrun_logs/peripherals_log.log");
    // control_ros_status_argv.push_back("2>&1 &");
    control_ros_status_argv.push_back("&");

    /*************************************************/

    driver_loader();
    sleep(5); // Wait for performance

    PAPI::system::runCommand_system(px4_cmd, px4_argv);
    sleep(5); // Wait for performance

    PAPI::system::runCommand_system(mavros_cmd, mavros_argv);
    sleep(5); // Wait for perfomance

    PAPI::system::runCommand_system(controller_cmd, controller_argv);
    sleep(5); // Wait for performance

    PAPI::system::runCommand_system(control_ros_status_cmd, control_ros_status_argv);
    sleep(1); // Wait for performace

    return true;
}

// Init the system, contain MAVROS and GEOMETRIC_CONTROLLER.
bool preInit()
{
    std::string mavros_cmd = "roslaunch";
    std::vector<std::string> mavros_argv;
    mavros_argv.push_back("mavros");
    mavros_argv.push_back("px4.launch");
    mavros_argv.push_back("fcu_url:=\"udp://:14540@localhost:14557\"");
    mavros_argv.push_back("> /home/pino/logs/roslaunch_logs/mavros_log.log");
    // mavros_argv.push_back("2>&1 &");
    mavros_argv.push_back("&");

    std::string controller_cmd = "roslaunch";
    std::vector<std::string> controller_argv;
    controller_argv.push_back("geometric_controller");
    controller_argv.push_back("automatic.launch");
    controller_argv.push_back("> /home/pino/logs/roslaunch_logs/controller_log.log");
    // controller_argv.push_back("2>&1 &");
    controller_argv.push_back("&");

    std::string control_ros_status_cmd = "rosrun";
    std::vector<std::string> control_ros_status_argv;
    control_ros_status_argv.push_back("control_pkg");
    control_ros_status_argv.push_back("automatic");
    control_ros_status_argv.push_back("> /home/pino/logs/rosrun_logs/control_log.log");
    // control_ros_status_argv.push_back("peripherals_status");
    // control_ros_status_argv.push_back("> /home/pino/logs/rosrun_logs/peripherals_log.log");
    // control_ros_status_argv.push_back("2>&1 &");
    control_ros_status_argv.push_back("&");

    /*************************************************/

    driver_loader();
    sleep(5); // Wait for performance

    PAPI::system::runCommand_system(mavros_cmd, mavros_argv);
    sleep(5); // Wait for perfomance

    PAPI::system::runCommand_system(controller_cmd, controller_argv);
    sleep(5); // Wait for performance

    PAPI::system::runCommand_system(control_ros_status_cmd, control_ros_status_argv);
    sleep(1); // Wait for performace

    return true;
}

// The main control is running in here, contain json parsing, handle problem, make mission, comm with others...
bool missionExecution()
{
    std::string mission_path = DEFAULT_MISSION_DIR_PATH;
    mission_path = mission_path + PAPI::system::getMissionFile(DEFAULT_MISSION_DIR_PATH);
    mission_path.pop_back();
    std::cout << mission_path << std::endl;
    if (!PAPI::system::jsonParsing(mission_path, mission))
        return false;

    PAPI::mission_id = mission.id;

    int index = 0;
    while (index < mission.number_sequence_items)
    {
        { /* Init Instruction Addition: */
            if (mission.sequence_names[index] == "init_sequence")
            {
                if (!InitSequenceAddition())
                {
                    PAPI::communication::sendMessage_echo_netcat("[ERROR] Fail to make Init Instruction.", DEFAULT_COMM_MSG_PORT);
                    PAPI::system::sleepLessThanASecond(0.1);
                    return false;
                }
            }
        }

        { /* Travel Instruction Addition: */
            if (mission.sequence_names[index] == "travel_sequence")
            {
                if (!TravelSequenceAddition())
                {
                    PAPI::communication::sendMessage_echo_netcat("[ERROR] Fail to make Travel Instruction.", DEFAULT_COMM_MSG_PORT);
                    PAPI::system::sleepLessThanASecond(0.1);
                    return false;
                }
            }
        }

        { /* Action Instruction Addition: */
            if (mission.sequence_names[index] == "action_sequence")
            {
                if (!ActionSequenceAddition())
                {
                    PAPI::communication::sendMessage_echo_netcat("[ERROR] Fail to make Action Instruction.", DEFAULT_COMM_MSG_PORT);
                    PAPI::system::sleepLessThanASecond(0.1);
                    return false;
                }
            }
        }

        if (!PAPI::drone::makeInstruction(mission.sequence_istructions[index]))
        {
            std::stringstream ss;
            ss << "[ERROR] Fail to make instruction no." << index << ": " << mission.sequence_istructions[index]->name << ".";
            PAPI::communication::sendMessage_echo_netcat(ss.str(), DEFAULT_COMM_MSG_PORT);
            PAPI::system::sleepLessThanASecond(0.1);
            return false;
        }

        { /* Travel Instruction Cleaner: */
            if (mission.sequence_names[index] == "travel_sequence")
            {
                if (!TravelInstructionCleaner(index))
                {
                    PAPI::communication::sendMessage_echo_netcat("[ERROR] Fail to clean Travel Instruction.", DEFAULT_COMM_MSG_PORT);
                    PAPI::system::sleepLessThanASecond(0.1);
                    return false;
                }
            }
        }

        { /* Action Instruction Cleaner: */
            if (mission.sequence_names[index] == "action_sequence")
            {
                if (!ActionInstructionCleaner(index))
                {
                    PAPI::communication::sendMessage_echo_netcat("[ERROR] Fail to clean Action Instruction.", DEFAULT_COMM_MSG_PORT);
                    PAPI::system::sleepLessThanASecond(0.1);
                    return false;
                }
            }
        }

        ++index;
    }

    return true;
}

// Close logs, sockets.
void close()
{
    PAPI::system::closeLogsFile();

    server_peripherals.serverClose();
    client_peripherals.clientClose();
}

int main()
{
    PAPI::system::createLogsFile(DEFAULT_LOG_DIR);

    if (!preInit())
    {
        // std::cerr << "MAVROS and GEOMETRIC_CONTROLLER initialization was unsuccessful." << std::endl;
        PAPI::communication::sendMessage_echo_netcat("[ERROR] MAVROS and GEOMETRIC_CONTROLLER initialization was unsuccessful.", DEFAULT_COMM_MSG_PORT);
        return -1;
    }
    // std::cout << "MAVROS and GEOMETRIC_CONTROLLER initialization was successful." << std::endl;
    PAPI::communication::sendMessage_echo_netcat("[ INFO] MAVROS and GEOMETRIC_CONTROLLER initialization was successful.", DEFAULT_COMM_MSG_PORT);

    if (!missionExecution())
    {
        // std::cerr << "The execution of the mission was unsuccessful." << std::endl;
        PAPI::communication::sendMessage_echo_netcat("[ERROR] The execution of the mission was unsuccessful.", DEFAULT_COMM_MSG_PORT);
        return -1;
    }
    // std::cout << "The mission has been successfully completed." << std::endl;
    PAPI::communication::sendMessage_echo_netcat("[ INFO] The mission has been successfully completed.", DEFAULT_COMM_MSG_PORT);

    close();

    return 0;
}