#include <ros/ros.h>
#include <string.h>

#include <custom_parameter/parameterHandler.h>
#include <custom_parameter/parameter.h>

#include "controller/KollrobotMoveGroup.h"

//common stuff
float iRefreshRate = 0.2;
std::string nodeName = "kollrobot_controller";

//ros sutff
ros::NodeHandle* _node;

//parameter stuff
customparameter::ParameterHandler* parameterHandler;
//customparameter::Parameter<bool> param_UseCurrentPoseForTask;


KollrobotMoveGroup* armGroup;

void InitParams()
{
    std::string subNamespace = "";
    //Standard params
    //parameterHandler->AddParameter("MapFrame", "", (std::string)"/map");   
}

float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, nodeName);
  _node = new ros::NodeHandle(nodeName);

  //init params
  parameterHandler = new customparameter::ParameterHandler(_node);
  InitParams();

  armGroup = new KollrobotMoveGroup(_node, parameterHandler, "arm");

  ros::Rate rate(iRefreshRate);
  while(_node->ok())
  {      
    //make new plan if armGroup is not planning
    if(!armGroup->IsPlanning && !armGroup->IsExecuting)
    {
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = 0.5;
        pose.position.y = RandomFloat(0.2, 0.6);
        pose.position.z = RandomFloat(-0.5, 0.5);
        armGroup->PlanToPoseExecute(pose);
        //armGroup->PlanToPose(pose);
    }

    //trigger the ros callbacks and wait the needed time
    ros::spinOnce();
    rate.sleep();

  }
  return 0;
}
