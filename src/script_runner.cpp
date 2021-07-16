#include "script_runner.h"
#include "ros_parameter.hpp"

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;


ScriptRunner::ScriptRunner(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
: _nh(nh)
{   
    LoadParam(private_nh, "script_path", _script_path); 

    std::string script_service = "run_script"; 
    _script_svr = _nh.advertiseService(script_service, &ScriptRunner::ScriptService, this);
    ROS_INFO_STREAM("Advertised script service: " << script_service); 
}

// run script service 
bool ScriptRunner::ScriptService(ros_shell::RunScript::Request &request,
                                 ros_shell::RunScript::Response &response)
{
    std::string script = request.script;
    std::string args = request.args;  
    std::string shell = request.shell; 
    ROS_INFO_STREAM("Script service request: " << script << " " << args);
    ROS_INFO_STREAM("Shell: " << shell); 

    if (script.empty())
    {
        ROS_WARN_STREAM("No script in request!"); 
        response.success = false;  
        response.message = "No script in request!"; 
        return false;
    }

    std::string script_file;
    script_file = _script_path.empty() ? "./" + script : _script_path + "/" + script; 
    if (!fs::exists(script_file)) 
    {
        ROS_WARN_STREAM("Script file not found: " << script_file); 
        response.success = false;  
        response.message = "Script file not found!"; 
        return false;
    }

    std::string script_ext = fs::path(script).extension(); 
    if (shell.empty()) {
        if (script_ext == ".py") shell = "python"; 
        else shell = "bash"; 
    }
    else {
        if ((script_ext == ".py" && shell.substr(0,6) != "python") || 
            (script_ext == ".sh" && (shell != "sh" || shell != "bash"))) 
        {
            ROS_INFO_STREAM("Invalid shell for " << script << " : " << shell); 
            response.success = false;  
            response.message = "Script and shell mismatching!"; 
            return false; 
        }
    }
    std::string command_line = shell + " " + script_file; 
    if (!args.empty()) command_line = command_line + " " + args; 
    command_line = command_line + " &"; 
    ROS_INFO_STREAM("Script command line: " << command_line);
    system(command_line.c_str());
    response.success = true;  
    return true; 
}
