#include <ros/ros.h>
#include "script_runner.h"
#include "command_runner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_shell");
    ros::NodeHandle nh, private_nh("~");
    ScriptRunner script_runner(nh, private_nh); 
    CommandRunner command_runner(nh, private_nh); 
    ros::spin();
    return 0;
}