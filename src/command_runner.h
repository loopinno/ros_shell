#ifndef __COMMAND_RUNNER_H
#define __COMMAND_RUNNER_H

#include <ros/ros.h>
#include <ros_shell/RunCommand.h>

class CommandRunner 
{
public: 
    CommandRunner(const ros::NodeHandle& nh = ros::NodeHandle(), 
                  const ros::NodeHandle& private_nh = ros::NodeHandle("~"));

private: 
    // run command service 
    bool CommandService(ros_shell::RunCommand::Request &request,
                        ros_shell::RunCommand::Response &response);

private: 
    ros::NodeHandle _nh;

    // run command server 
    ros::ServiceServer _command_svr; 
};

#endif // ifndef __COMMAND_RUNNER_H
