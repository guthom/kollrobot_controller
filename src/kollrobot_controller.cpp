#include <ros/ros.h>
#include <string.h>

#include <custom_parameter/parameterHandler.h>
#include <custom_parameter/parameter.h>

//common stuff
int iRefreshRate = 30;
std::string nodeName = "kollrobot_controller";

//ros sutff
ros::NodeHandle* node;

//parameter stuff
customparameter::ParameterHandler* parameterHandler;
//customparameter::Parameter<bool> param_UseCurrentPoseForTask;


void InitParams()
{
    std::string subNamespace = "";
    //Standard params
    //parameterHandler->AddParameter("MapFrame", "", (std::string)"/map");   
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, nodeName);
  node = new ros::NodeHandle(nodeName);

  //init params
  parameterHandler = new customparameter::ParameterHandler(node);
  InitParams();

  ros::Rate rate(iRefreshRate);
  while(node->ok())
  {      
    //trigger the ros callbacks and wait the needed time
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
