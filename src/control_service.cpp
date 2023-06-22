/* Compile:
    g++ control_service.cpp -o control_service -pthread -ljsoncpp
 */
#include "/home/pino/pino_ws/papi/PAPI.h"

MissionRequest mission;

PAPI::communication::Server server_peripherals(DEFAULT_PERIPHERALS_STATUS_CONTROL_PORT);
PAPI::communication::Client client_peripherals(LOCAL_HOST, DEFAULT_PERIPHERALS_STATUS_NODE_PORT);

// [DEMO] Init the system, contain PX4, MAVROS and GEOMETRIC_CONTROLLER.
bool demo()
{
    std::string px4_cmd = "roslaunch";
    std::vector<std::string> px4_argv;
    px4_argv.push_back("px4");
    px4_argv.push_back("posix_sitl.launch");
    px4_argv.push_back("> /home/pino/logs/roslaunch_logs/px4_log.log");
    // px4_argv.push_back("2>&1 &");
    px4_argv.push_back("2>&1");

    std::string mavros_cmd = "roslaunch";
    std::vector<std::string> mavros_argv;
    mavros_argv.push_back("mavros");
    mavros_argv.push_back("px4.launch");
    mavros_argv.push_back("fcu_url:=\"udp://:14540@localhost:14557\"");
    mavros_argv.push_back("> /home/pino/logs/roslaunch_logs/mavros_log.log");
    mavros_argv.push_back("2>&1 &");

    std::string controller_cmd = "roslaunch";
    std::vector<std::string> controller_argv;
    controller_argv.push_back("geometric_controller");
    controller_argv.push_back("automatic.launch");
    controller_argv.push_back("> /home/pino/logs/roslaunch_logs/controller_log.log");
    controller_argv.push_back("2>&1 &");

    std::string peripherals_status_cmd = "rosrun";
    std::vector<std::string> peripherals_status_argv;
    peripherals_status_argv.push_back("peripherals_status");
    peripherals_status_argv.push_back("automatic");
    peripherals_status_argv.push_back("> /home/pino/logs/rosrun_logs/peripherals_log.log");
    peripherals_status_argv.push_back("2>&1 &");

    /*************************************************/

    PAPI::system::runCommand_system(px4_cmd, px4_argv);
    sleep(5); // Wait for performance

    PAPI::system::runCommand_system(mavros_cmd, mavros_argv);
    sleep(5); // Wait for perfomance

    PAPI::system::runCommand_system(controller_cmd, controller_argv);
    sleep(5); // Wait for performance

    PAPI::system::runCommand_system(peripherals_status_cmd, peripherals_status_argv);
    sleep(1); // Wait for performace

    /*************************************************/

    std::cout << "START CONNECTING." << std::endl;
    int connection_result = (client_peripherals.clientStart() == -1 || server_peripherals.serverStart() == -1) ? -1 : 0;
    return (connection_result == -1) ? false : true;
}

// Init the system, contain MAVROS and GEOMETRIC_CONTROLLER.
bool preInit()
{
    std::string mavros_cmd = "roslaunch";
    std::vector<std::string> mavros_argv;
    mavros_argv.push_back("mavros");
    mavros_argv.push_back("px4.launch");
    mavros_argv.push_back("> /home/pino/logs/roslaunch_logs/mavros_log.log");
    // mavros_argv.push_back("2>&1 &");

    std::string controller_cmd = "roslaunch";
    std::vector<std::string> controller_argv;
    controller_argv.push_back("geometric_controller");
    controller_argv.push_back("automatic.launch");
    controller_argv.push_back("> /home/pino/logs/roslaunch_logs/controller_log.log");
    // controller_argv.push_back("2>&1 &");

    std::string peripherals_status_cmd = "rosrun";
    std::vector<std::string> peripherals_status_argv;
    peripherals_status_argv.push_back("peripherals_status");
    peripherals_status_argv.push_back("automatic");
    peripherals_status_argv.push_back("> /home/pino/logs/rosrun_logs/peripherals_log.log");
    // peripherals_status_argv.push_back("2>&1 &");

    /*************************************************/

    PAPI::system::runCommand_system(mavros_cmd, mavros_argv);
    sleep(5); // Wait for perfomance

    PAPI::system::runCommand_system(controller_cmd, controller_argv);
    sleep(5); // Wait for performance

    PAPI::system::runCommand_system(peripherals_status_cmd, peripherals_status_argv);
    sleep(1); // Wait for performace

    /*************************************************/

    std::cout << "START CONNECTING." << std::endl;
    int connection_result = (client_peripherals.clientStart() == -1 || server_peripherals.serverStart() == -1) ? -1 : 0;
    return (connection_result == -1) ? false : true;
}

bool initCheck()
{
    std::string first_msg = "";
    // Record the starting time
    auto startTime_firstStatus = std::chrono::high_resolution_clock::now();
    PAPI::communication::sendMessage_echo_netcat("[ INFO] Waiting for first status message...", DEFAULT_COMM_MSG_PORT);
    do
    {
        first_msg = client_peripherals.reciveMessage();
        // std::cout << std::endl
        //           << first_msg << std::endl;
        startTime_firstStatus = std::chrono::high_resolution_clock::now();
    } while (first_msg.empty());

    // Set the timeout duration
    auto wait_for_active_duration = std::chrono::seconds(DEFAULT_TIME_WAIT_FOR_ACTIVE);
    std::chrono::seconds elapsed_duration;
    std::string last_msg = client_peripherals.reciveMessage();

    PAPI::communication::sendMessage_echo_netcat("[ INFO] Waiting for final status result...", DEFAULT_COMM_MSG_PORT);
    do
    {
        // Check if the timeout has occurred
        auto currentTime = std::chrono::high_resolution_clock::now();
        elapsed_duration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime_firstStatus);

        last_msg = client_peripherals.reciveMessage();

    } while (elapsed_duration < wait_for_active_duration);

    std::cout << "Last status message: " << last_msg << std::endl;

    std::string mav_state;
    std::vector<int> peripherals_status_vector = PAPI::system::getPeripheralsStatus_fromString_toVector(last_msg, mav_state);

    std::vector<int> peripherals_list;
    mission.sequence_istructions[0]->Init_getPeripherals(peripherals_list);

    std::cout << "PASS #1.\n"; /************************************/

    auto it = std::find(peripherals_list.begin(), peripherals_list.end(), Peripheral::PERIPHERAL_CAM_DOWNWARD);
    if (it != peripherals_list.end() && peripherals_status_vector[DEVICE::FLIR] == PERIPHERAL_STATUS::ACTIVE)
    {
        PAPI::system::sendImage(Peripheral::PERIPHERAL_CAM_DOWNWARD);
        PAPI::communication::sendMessage_echo_netcat("[ INFO] FLIR image sended.", DEFAULT_COMM_MSG_PORT);
    }
    else
        PAPI::communication::sendMessage_echo_netcat("[ERROR] FLIR camera is not activated and will not send the image to GCS.", DEFAULT_COMM_MSG_PORT);
    PAPI::system::sleepLessThanASecond(0.1);

    it = std::find(peripherals_list.begin(), peripherals_list.end(), Peripheral::PERIPHERAL_CAM_FORWARD);
    if (it != peripherals_list.end() && peripherals_status_vector[DEVICE::D455] == PERIPHERAL_STATUS::ACTIVE)
    {
        PAPI::system::sendImage(Peripheral::PERIPHERAL_CAM_FORWARD);
        PAPI::communication::sendMessage_echo_netcat("[ INFO] D455 image sended.", DEFAULT_COMM_MSG_PORT);
    }
    else
        PAPI::communication::sendMessage_echo_netcat("[ERROR] D455 camera is not activated and will not send the image to GCS.", DEFAULT_COMM_MSG_PORT);
    PAPI::system::sleepLessThanASecond(0.1);

    PAPI::communication::sendMessage_echo_netcat(PAPI::system::statusParsing(last_msg), DEFAULT_COMM_MSG_PORT);
    PAPI::system::sleepLessThanASecond(0.1);

    std::cout << "PASS #2.\n"; /************************************/

    std::string confirm_msg = "";                                                              // Confirm message
    auto wait_for_image_confirm_timeout = std::chrono::seconds(DEFALUT_IMAGE_CONFIRM_TIMEOUT); // Set the timeout duration
    auto sendTime = std::chrono::high_resolution_clock::now();                                 // Time after send image(s)
    PAPI::communication::sendMessage_echo_netcat("[ INFO] Waiting for confirm from GCS.", DEFAULT_COMM_MSG_PORT);
    PAPI::system::sleepLessThanASecond(0.1);
    do
    { // Check if the timeout has occurred
        auto currentTime = std::chrono::high_resolution_clock::now();
        elapsed_duration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - sendTime);
        confirm_msg = PAPI::system::readLastLineFromFile(DEFAULT_MESSAGE_FILE_PATH);

    } while (elapsed_duration < wait_for_image_confirm_timeout && confirm_msg.empty());

    std::cout << "PASS #3.\n"; /************************************/

    if (confirm_msg.empty())
    {
        PAPI::communication::sendMessage_echo_netcat("[ERROR] Stop Init: Wait for image check TIMEOUT.", DEFAULT_COMM_MSG_PORT);
        PAPI::system::sleepLessThanASecond(0.1);
        return false;
    }
    else
    {
        PAPI::communication::sendMessage_echo_netcat("[ INFO] Got confirm from GCS.", DEFAULT_COMM_MSG_PORT);
        PAPI::system::sleepLessThanASecond(0.1);
    }

    if (!PAPI::system::checkFLAG(confirm_msg))
    {
        PAPI::communication::sendMessage_echo_netcat("[ERROR] Stop Init: Reject from GCS.", DEFAULT_COMM_MSG_PORT);
        PAPI::system::sleepLessThanASecond(0.1);
        return false;
    }
    else
    {
        PAPI::communication::sendMessage_echo_netcat("[ INFO] Allowed from GCS.", DEFAULT_COMM_MSG_PORT);
        PAPI::system::sleepLessThanASecond(0.1);
    }

    if (!PAPI::drone::peripheralsCheck(peripherals_status_vector))
    {
        PAPI::communication::sendMessage_echo_netcat("[ERROR] Stop Init: Peripherals check failed.", DEFAULT_COMM_MSG_PORT);
        PAPI::system::sleepLessThanASecond(0.1);
        return false;
    }
    else
    {
        PAPI::communication::sendMessage_echo_netcat("[ INFO] Peripherals check passed. Everything is working.", DEFAULT_COMM_MSG_PORT);
        PAPI::system::sleepLessThanASecond(0.1);
    }

    return true;
}

// The main control is running in here, contain json parsing, handle problem, make mission, comm with others...
bool missionExecution()
{
    if (!PAPI::system::jsonParsing(DEFAULT_JSON_FILE_PATH, mission))
        return false;

    int index = 0;
    while (index < mission.number_sequence_items)
    {
        { /* Init Instruction Addition: */
            if (index == 0)
            {
                /* Peripherals usage status change: */
                std::vector<int> list;
                mission.sequence_istructions[0]->Init_getPeripherals(list);
                std::stringstream ss;
                for (auto i = 0; i < list.size(); i++)
                    ss << list[i] << "|";
                server_peripherals.sendMsg(ss.str());
                // std::cout << std::endl
                //           << ss.str() << std::endl;

                /************************************/

                /* Send Image, Status and get Response: */
                if (!initCheck())
                {
                    PAPI::communication::sendMessage_echo_netcat("[ERROR] Fail to make Init Instruction.", DEFAULT_COMM_MSG_PORT);
                    return false;
                }
            }
        }

        if (!PAPI::drone::makeInstruction(mission.sequence_istructions[index]))
        {
            std::stringstream ss;
            ss << "[ERROR] Fail to make instruction no." << index << ": " << mission.sequence_istructions[index]->name;
            PAPI::communication::sendMessage_echo_netcat(ss.str(), DEFAULT_COMM_MSG_PORT);

            return false;
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