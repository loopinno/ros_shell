#include "command_runner.h"
#include "ros_parameter.hpp"

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;


CommandRunner::CommandRunner(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
: _nh(nh)
{   
    std::string command_service = "run_command"; 
    _command_svr = _nh.advertiseService(command_service, &CommandRunner::CommandService, this);
    ROS_INFO_STREAM("Advertised command service: " << command_service); 
}

// run command service 
bool CommandRunner::CommandService(ros_shell::RunCommand::Request &request,
                                   ros_shell::RunCommand::Response &response)
{
    std::string command = request.command;
    ROS_INFO_STREAM("Command service request: " << command);

    if (command.empty())
    {
        ROS_WARN_STREAM("No command in request!"); 
        response.success = false;  
        response.message = "No command in request!"; 
        return false;
    }

    ROS_INFO_STREAM("Command line: " << command);
    system(command.c_str());
    response.success = true;  
    return true; 
}
