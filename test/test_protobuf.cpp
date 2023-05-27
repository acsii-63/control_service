#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <ios>

#include "mission.pb.h"

// Input binary file get from communication service, contain RequestMission
std::fstream input;

// Fly Mission is store in SendMissionRequest class object
mission::v1::SendMissionRequest mission_request;

bool display(const mission::v1::SendMissionRequest &_mission_request)
{
    return true;
}

// Parsing Mission from binary file, return true if success, false if not.
bool parsingMission()
{
    if (!mission_request.ParseFromIstream(&input))
    {
        std::cerr << std::endl
                  << "****************************" << std::endl
                  << "*   UNSUCCESSFUL PARSING   *" << std::endl
                  << "****************************" << std::endl;
        return false;
    }

    std::cout << std::endl
              << "****************************" << std::endl
              << "*    SUCCESSFUL PARSING    *" << std::endl
              << "****************************" << std::endl;
    return true;
}

int main(int argc, char *argv[])
{
    // Verify that the version of the library that we linked against is
    // compatible with the version of the headers we compiled against.
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    input.open(argv[1], std::ios::in | std::ios::binary);

    // If parsing fail, exit with code -1
    if (!parsingMission())
        return -1;

    std::cout << "Number of Items in Sequence: " << mission_request.sequence_items_size() << std::endl;


    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}