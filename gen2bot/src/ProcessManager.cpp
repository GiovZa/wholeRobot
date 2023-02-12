#include <gen2bot/ProcessManager.h>
#include "ProcessManager.h"
#include "std_msgs/Int8.h"

ProcessManager::ProcessManager(int* p_cmd)
{   
    this->p_cmd = p_cmd; // nothing
}

void ProcessManager::callback(const std_msgs::Int8ConstPtr &msg)
{
    ROS_INFO("Callback");
    int* p = this->p_cmd;
    *p = msg->data;
}
