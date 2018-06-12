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
    void PlanSimulationPath();
    void UpdateCurrentState();
    void MoveToValidRandom();
    void Execute();

    moveit::planning_interface::PlanningSceneInterface* _planningSzene;
private:
    //node stuff
    ros::NodeHandle* _node;
    std::string _nodeName;
    void Run();
    boost::thread* _nodeThread;

    std::string _groupName;
    void Init(ros::NodeHandle* parentNode);
    void InitParameter();

    void RunPlanning();
    void RunPlanningExecute();
    void MoveToValidRandomRun();
    boost::thread* _planningThread;

    //publisher
    void PublishMarker();
    ros::Publisher _pubTargetPose;
    void InitMarker();
    visualization_msgs::Marker _markerTargetPose;


    //moveIt Stuff
    moveit::planning_interface::MoveGroupInterface* _moveGroup;
    moveit::planning_interface::MoveGroupInterface::Plan _currentPlan;
    moveit::planning_interface::MoveGroupInterface::Plan _currentExecutedPlan;

    //parameter stuff
    void InitRosParams();
    customparameter::ParameterHandler* _parameterHandler;
    customparameter::Parameter<int> _param_RefreshRate;
};

#endif // KOLLROBOTMOVEGROUP_H
