/* Compile:
    g++ control_service.cpp -o control_service -pthread -ljsoncpp
 */

#include "../../papi/PAPI.h"

MissionRequest mission;

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

    /*************************************************/

    PAPI::system::runCommand_system(px4_cmd, px4_argv);
    sleep(15); // Low-end System :)

    PAPI::system::runCommand_system(mavros_cmd, mavros_argv);
    sleep(20); // Low-end System :)

    PAPI::system::runCommand_system(controller_cmd, controller_argv);
    sleep(5); // Low-end System :)

    return true;
}

bool missionExecution()
{
    if (!PAPI::system::jsonParsing("/home/pino/pino_ws/papi/sample/sample_takeoff_land.json", mission))
        return false;

    int index = 0;
    while (index < mission.number_sequence_items)
    {
        if (!PAPI::drone::makeInstruction(mission.sequence_istructions[index]))
        {
            std::cerr << "Fail to make instruction no." << index << ": " << mission.sequence_istructions[index]->name << std::endl;
            return false;
        }
        ++index;
    }

    return true;
}

int main()
{
    PAPI::system::createLogsFile("../logs");

    if (!initWorldAndVehicle())
    {
        std::cerr << "Initialization of the World, Vehicle, and Controller was unsuccessful." << std::endl;
        return -1;
    }
    std::cout << "Initialization of the World, Vehicle, and Controller was successful." << std::endl;

    if (!missionExecution())
    {
        std::cerr << "The execution of the mission was unsuccessful." << std::endl;
        return -1;
    }
    std::cout << "The mission has been successfully completed." << std::endl;

    PAPI::system::closeLogsFile();

    return 0;
}
