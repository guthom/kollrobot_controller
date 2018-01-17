#include <ros/ros.h>
#include <stdlib.h>
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
customparameter::Parameter<float> paramMaxWorkspace;
customparameter::Parameter<bool> paramSimMode;
customparameter::Parameter<bool> paramDemoMode;
customparameter::Parameter<bool> paramDemoUR10Mode;
customparameter::Parameter<bool> paramNormalMode;

KollrobotMoveGroup* armGroup;

void InitParams()
{
    std::string subNamespace = "";
    //Standard params
    paramMaxWorkspace = parameterHandler->AddParameter("MaxWorkspace", "", 0.8f);
    paramSimMode = parameterHandler->AddParameter("SimMode", "", false);
    paramDemoMode = parameterHandler->AddParameter("DemoMode", "", false);
    paramDemoUR10Mode = parameterHandler->AddParameter("DemoUR10Mode", "", true);
    paramNormalMode = parameterHandler->AddParameter("NormalMode", "", false);
}

float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

void RunDemoMode()
{
    armGroup = new KollrobotMoveGroup(_node, parameterHandler, "arm");

    ros::Rate rate(iRefreshRate);
    while(_node->ok())
    {
        armGroup->UpdateCurrentState();
        //make new plan if armGroup is not planning
        if(!armGroup->IsPlanning && !armGroup->IsExecuting)
        {
            geometry_msgs::Pose pose;
            pose.orientation.w = 1;
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;

            // calculate random pose according to a really simple model of the workspace
            // neoboix box size ~"0.70 0.60 0.20"

            float maxWorkspace = paramMaxWorkspace.GetValue();
            float xOffset = 0.35;
            float yOffset = 0.30;

            //left or right
            pose.position.y = RandomFloat(yOffset, maxWorkspace);
            int leftRight = rand() % 2;
            if(leftRight == 0)
            {
                //switch sign for left area
                pose.position.y = -pose.position.y;
            }

            //front or back
            pose.position.x = RandomFloat(xOffset, maxWorkspace);
            int frontBack = rand() % 2;
            if(frontBack == 0)
            {
                //switch sign for back area
                pose.position.x = -pose.position.x;
            }

            //finally send random pose
            pose.position.z = RandomFloat(0.2, maxWorkspace);

            ROS_INFO_STREAM("Sent new Pose:" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z);
            armGroup->PlanToPoseExecute(pose);
            //armGroup->PlanToPose(pose);
        }

        //trigger the ros callbacks and wait the needed time
        ros::spinOnce();
        rate.sleep();
    }
}

void RunDemoUR10Mode()
{
    armGroup = new KollrobotMoveGroup(_node, parameterHandler, "manipulator");

    ros::Rate rate(iRefreshRate);
    while(_node->ok())
    {
        armGroup->UpdateCurrentState();
        //make new plan if armGroup is not planning
        if(!armGroup->IsPlanning && !armGroup->IsExecuting)
        {
            armGroup->MoveToValidRandom();
        }

        //trigger the ros callbacks and wait the needed time
        ros::spinOnce();
        rate.sleep();
    }
}

void RunSimMode()
{
    armGroup = new KollrobotMoveGroup(_node, parameterHandler, "arm");

    ros::Rate rate(iRefreshRate);
    while(_node->ok())
    {
        if(!armGroup->IsExecuting)
        {
            armGroup->PlanSimulationPath();
            armGroup->Execute();
        }
    }
}

void RunNormalMode()
{
    ros::Rate rate(iRefreshRate);
    while(_node->ok())
    {
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);
    _node = new ros::NodeHandle(nodeName);

    //init params
    parameterHandler = new customparameter::ParameterHandler(_node);
    InitParams();

    if (paramDemoMode.GetValue())
        RunDemoMode();


    if (paramDemoUR10Mode.GetValue())
        RunDemoUR10Mode();

    if (paramSimMode.GetValue())
        RunSimMode();

    if (paramNormalMode.GetValue())
        RunNormalMode();

  return 0;
}
