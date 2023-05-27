#include <cstdlib>
#include <cstdbool>
#include <unistd.h>
#include <sys/wait.h>

#include <iostream>
#include <vector>
#include <string>

// Get the PID of a node using pgrep.
int getPID_pgrep(const std::string &_node_name)
{
    std::string command_system = "pgrep -f " + _node_name; // Contruct the command to run without ROS master
    int pid_system;

    FILE *pipe;
    std::string pid_str = "";
    char buffer[128];
    std::string result = "";

    // Open a pipe to run the command and read the output
    pipe = popen(command_system.c_str(), "r");
    if (!pipe)
    {
        std::cerr << "Failed to run command: " << command_system << std::endl;
        return -1;
    }

    while (!feof(pipe))
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;

    // if (result.empty())
    //     return -1;

    // Extract the system PID from the output
    pid_str = result.substr(0, result.find_first_of("\r\n"));
    pid_system = std::stoi(pid_str);

    pclose(pipe);

    return pid_system;
}

// Get the PID of a node.
int getPID(const std::string &_node_name)
{
    std::string node_name = _node_name; // The name of the node to get the PID of

    std::string command_master = "rosnode info " + node_name + " | grep Pid"; // Construct the command to run with ROS master

    char buffer[128];
    std::string result = "";

    std::string pid_str = "";
    int pid_master;
    FILE *pipe;

    // Open a pipe to run the command and read the output
    pipe = popen(command_master.c_str(), "r");
    if (!pipe)
    {
        std::cerr << "Failed to run command: " << command_master << std::endl;
        return getPID_pgrep(_node_name);
    }

    while (!feof(pipe))
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;

    // Extract the ERROR from the output
    size_t error = result.find("ERROR");
    if (error != std::string::npos)
        return getPID_pgrep(_node_name);

    // Extract the system PID from the output
    pid_str = result.substr(result.find(":") + 2, result.length() - 1);
    pid_master = std::stoi(pid_str);

    return pid_master;
}

// Check status of a node using pid status.
bool checkStatus_PID(const int _pid)
{
    return _pid > 0 ? true : false;
}

// Run a command with given argv in std::vector<std::string> type;
void runCommand(const std::string _command, const std::vector<std::string> _argv)
{
    pid_t pid = fork();
    if (pid == 0)
    {
        /* Child process */
        std::string command = _command;
        char *cmd = const_cast<char *>(command.c_str()); // const char *__file

        char **argv = new char *[_argv.size() + 2]; // char *const *__argv
        argv[0] = const_cast<char *>(command.c_str());
        for (int i = 0; i < _argv.size(); i++)
            argv[i + 1] = const_cast<char *>(_argv[i].c_str());
        argv[_argv.size() + 1] = NULL;

        execvp(cmd, argv);

        std::cerr << "Failed to execute " << cmd << "." << std::endl;

        exit(1);
    }
    else if (pid > 0)
    {
        /* Parent process */
        // std::cout << "Command started in process" << pid << "." << std::endl;
    }
    else
    {
        // Fork failed
        std::cerr << "Failed to fork." << std::endl;
        exit(1);
    }

    /* The code after fork() call goes here */
}

// Kill a node by kill the process
void killNode_system(const std::string _node_name)
{
    std::string pid_str = std::to_string(getPID(_node_name));
    std::string kill_command = "kill -9 " + pid_str;

    std::cout << "KILL SYSTEM" << std::endl;
    system(kill_command.c_str());
}

// Kill a node by it's name
void killNode_name(const std::string _node_name)
{
    std::string kill_command_rosnode;

    kill_command_rosnode = "rosnode kill /" + _node_name;

    FILE *pipe = popen(kill_command_rosnode.c_str(), "r");
    if (!pipe)
    {
        // If the rosnode command dont run, kill node by kill command
        killNode_system(_node_name);
        return;
    }

    char buffer[128];
    std::string result = "";
    while (!feof(pipe))
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;

    // Extract the ERROR from the output, if get ERROR, kill node by kill command
    size_t error = result.find("ERROR");
    if (error != std::string::npos)
        killNode_system(_node_name);

    pclose(pipe);
}

// Get list of active node
std::vector<std::string> getNodeList()
{
    std::vector<std::string> result;

    std::string command = "rosnode list";
    FILE *pipe = popen(command.c_str(), "r");

    if (!pipe)
    {
        std::cerr << "Failed to execute " << command << "." << std::endl;
        return result;
    }

    char buffer[8192];
    while (fgets(buffer, 8192, pipe))
    {
        std::string temp_string = std::string(buffer);
        result.push_back(temp_string.substr(1, temp_string.size() - 2));
    }
    // result.emplace_back(buffer);

    return result;
}
