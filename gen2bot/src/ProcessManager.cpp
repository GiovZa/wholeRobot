#include <gen2bot/ProcessManager.h>
#include "ProcessManager.h"
#include "std_msgs/Int8.h"


// constuctor
ProcessManager::ProcessManager(int* p_cmd)
{   
    this->p_cmd = p_cmd; // assigns object's variable p_cmd to global variable p_cmd
}

// The function that always gets called on by subscriber
// Updates the variable based on what publisher (notDTTalker.py) spews out
void ProcessManager::callback(const std_msgs::Int8ConstPtr &msg)
{
    ROS_INFO("Callback");
    int* p = this->p_cmd;
    *p = msg->data;
}
