#ifndef KOLLROBOTMOVEGROUP_H
#define KOLLROBOTMOVEGROUP_H

#include <ros/ros.h>
#include <string.h>

#include <boost/thread.hpp>

#include <custom_parameter/parameterHandler.h>
#include <custom_parameter/parameter.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class KollrobotMoveGroup
{
public:
    KollrobotMoveGroup(ros::NodeHandle* parentNode,
                        customparameter::ParameterHandler* parameterHandler,
                       std::string groupName);

    ~KollrobotMoveGroup();

    bool IsPlanning = false;
    bool PlanValid = false;
    bool IsExecuting = false;

    void PlanToPose(geometry_msgs::Pose targetPose);
    void PlanToPoseExecute(geometry_msgs::Pose targetPose);
    void Execute();

private:
    //node stuff
    float _refreshRate = 30;
    ros::NodeHandle* _node;
    std::string _nodeName;
    void Run();
    boost::thread* _nodeThread;

    std::string _groupName;
    void Init(ros::NodeHandle* parentNode);

    void RunPlanning();
    void RunPlanningExecute();
    boost::thread* _planningThread;

    //publisher
    ros::Publisher _pubTargetPose;
    void InitMarker();
    visualization_msgs::Marker _markerTargetPose;

    //moveIt Stuff
    moveit::planning_interface::MoveGroupInterface* _moveGroup;
    moveit::planning_interface::PlanningSceneInterface* _planningSzene;
    moveit::planning_interface::MoveGroupInterface::Plan _currentPlan;
    moveit::planning_interface::MoveGroupInterface::Plan _currentExecutedPlan;

    //parameter stuff
    customparameter::ParameterHandler* _parameterHandler;
    void InitRosParams();
    //customparameter::Parameter<float> param_VisuUpdateRate;
};

#endif // KOLLROBOTMOVEGROUP_H
