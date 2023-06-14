/* Compile:
    g++ control_service.cpp -o control_service -pthread -ljsoncpp
 */

#include "../../papi/PAPI.h"

MissionRequest mission;

PAPI::communication::Server server(DEFAULT_PERIPHERALS_STATUS_CONTROL_PORT);
PAPI::communication::Client client(LOCAL_HOST, DEFAULT_PERIPHERALS_STATUS_NODE_PORT);

// Init the system, contain px4, mavros and geometric_controller
bool initWorldAndVehicle()
{
    std::string px4_cmd = "roslaunch";
    std::vector<std::string> px4_argv;
    px4_argv.push_back("px4");
    px4_argv.push_back("posix_sitl.launch");
    px4_argv.push_back("> /home/pino/logs/roslaunch_logs/px4_log.log");
    // px4_argv.push_back("2>&1 &");

    std::string mavros_cmd = "roslaunch";
    std::vector<std::string> mavros_argv;
    mavros_argv.push_back("mavros");
    mavros_argv.push_back("px4.launch");
    mavros_argv.push_back("fcu_url:=\"udp://:14540@localhost:14557\"");
    mavros_argv.push_back("> /home/pino/logs/roslaunch_logs/mavros_log.log");
    // mavros_argv.push_back("2>&1 &");

    std::string controller_cmd = "roslaunch";
    std::vector<std::string> controller_argv;
    controller_argv.push_back("geometric_controller");
    controller_argv.push_back("automatic.launch");
    controller_argv.push_back("> /home/pino/logs/roslaunch_logs/controller_log.log");
    // controller_argv.push_back("2>&1 &");

    std::string realsense_cmd = "roslaunch";
    std::vector<std::string> realsense_argv;
    realsense_argv.push_back("realsense2_camera");
    realsense_argv.push_back("rs_d400_and_t265.launch");
    realsense_argv.push_back("> /home/pino/logs/roslaunch_logs/realsense_log.log");
    // realsense_argv.push_back("2>&1 &");

    std::string spinnaker_cmd = "roslaunch";
    std::vector<std::string> spinnaker_argv;
    spinnaker_argv.push_back("spinnaker_camera_driver");
    spinnaker_argv.push_back("color_cam.launch");
    spinnaker_argv.push_back("> /home/pino/logs/roslaunch_logs/spinnaker_log.log");
    // spinnaker_argv.push_back("2>&1 &");

    /*************************************************/

    PAPI::system::runCommand_system(px4_cmd, px4_argv);
    sleep(15); // Wait for performance

    PAPI::system::runCommand_system(mavros_cmd, mavros_argv);
    sleep(10); // Wait for perfomance

    PAPI::system::runCommand_system(controller_cmd, controller_argv);
    sleep(5); // Wait for performance

    PAPI::system::runCommand_system(realsense_cmd, realsense_argv);
    sleep(5); // Wait for performace

    PAPI::system::runCommand_system(spinnaker_cmd, spinnaker_argv);
    sleep(5); // Wait for performace

    return true;
}

bool preInit()
{
    std::string mavros_cmd = "roslaunch";
    std::vector<std::string> mavros_argv;
    mavros_argv.push_back("mavros");
    mavros_argv.push_back("px4.launch");
    mavros_argv.push_back("fcu_url:=\"udp://:14540@localhost:14557\"");
    mavros_argv.push_back("> /home/pino/logs/roslaunch_logs/mavros_log.log");
    // mavros_argv.push_back("2>&1 &");

    std::string controller_cmd = "roslaunch";
    std::vector<std::string> controller_argv;
    controller_argv.push_back("geometric_controller");
    controller_argv.push_back("automatic.launch");
    controller_argv.push_back("> /home/pino/logs/roslaunch_logs/controller_log.log");
    // controller_argv.push_back("2>&1 &");

    std::string realsense_cmd = "roslaunch";
    std::vector<std::string> realsense_argv;
    realsense_argv.push_back("realsense2_camera");
    realsense_argv.push_back("rs_d400_and_t265.launch");
    realsense_argv.push_back("> /home/pino/logs/roslaunch_logs/realsense_log.log");
    // realsense_argv.push_back("2>&1 &");

    std::string spinnaker_cmd = "roslaunch";
    std::vector<std::string> spinnaker_argv;
    spinnaker_argv.push_back("spinnaker_camera_driver");
    spinnaker_argv.push_back("color_cam.launch");
    spinnaker_argv.push_back("> /home/pino/logs/roslaunch_logs/spinnaker_log.log");
    // spinnaker_argv.push_back("2>&1 &");

    PAPI::system::runCommand_system(mavros_cmd, mavros_argv);
    sleep(10); // Wait for perfomance

    PAPI::system::runCommand_system(controller_cmd, controller_argv);
    sleep(5); // Wait for performance

    PAPI::system::runCommand_system(realsense_cmd, realsense_argv);
    sleep(5); // Wait for performace

    PAPI::system::runCommand_system(spinnaker_cmd, spinnaker_argv);
    sleep(5); // Wait for performace

    std::cout << "START CONNECTING." << std::endl;
    int connection_result = (client.clientStart() == -1 || server.serverStart() == -1) ? -1 : 0;
    return (connection_result == -1) ? false : true;
}

bool missionExecution()
{
    if (!PAPI::system::jsonParsing("/home/pino/pino_ws/papi/sample/sample_takeoff_land.json", mission))
        return false;

    int index = 0;
    while (index < mission.number_sequence_items)
    {
        if (index == 0)
        {
            std::vector<int> list;
            mission.sequence_istructions[0]->Init_getPeripherals(list);
            std::stringstream ss;
            for (auto i = 0; i < list.size(); i++)
                ss << list[i] << "|";
            server.sendMsg(ss.str());
            std::cout << ss.str() << std::endl;
        }

        if (!PAPI::drone::makeInstruction(mission.sequence_istructions[index]))
        {
            std::cerr << "Fail to make instruction no." << index << ": " << mission.sequence_istructions[index]->name << std::endl;
            return false;
        }
        ++index;
    }

    return true;
}

void close()
{
    PAPI::system::closeLogsFile();

    server.serverClose();
    client.clientClose();
}

int main()
{
    PAPI::system::createLogsFile("../logs");

    // if (!initWorldAndVehicle())
    // {
    //     std::cerr << "Initialization of the World, Vehicle, and Controller was unsuccessful." << std::endl;
    //     return -1;
    // }
    // std::cout << "Initialization of the World, Vehicle, and Controller was successful." << std::endl;

    if (!preInit())
    {
        std::cerr << "Initialization was unsuccessful." << std::endl;
        return -1;
    }
    std::cout << "Initialization was successful." << std::endl;

    if (!missionExecution())
    {
        std::cerr << "The execution of the mission was unsuccessful." << std::endl;
        return -1;
    }
    std::cout << "The mission has been successfully completed." << std::endl;

    close();

    return 0;
}
