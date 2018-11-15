#pragma once

#include <ros/ros.h>
#include <cstring>

#include <boost/thread.hpp>

#include <custom_parameter/parameterHandler.h>
#include <custom_parameter/parameter.h>
#include <vector>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "../helper/TransformationHandler.h"
#include <visualization_msgs/MarkerArray.h>

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
    bool IsBusy();

    void PlanToPose(geometry_msgs::Pose targetPose);
    void PlanToPose(geometry_msgs::PoseStamped targetPose);
    void PlanToPoseExecute(geometry_msgs::Pose targetPose);
    void PlanToPoseExecute(geometry_msgs::PoseStamped targetPose);
    void PlanToPositionExecute(geometry_msgs::PointStamped targetPose);
    void PlanToPositionExecute(geometry_msgs::Point targetPose);
    moveit_msgs::RobotTrajectory ComputeCartesianpath(std::vector<geometry_msgs::Pose> waypoints,
                                                      std::string frameID);
    void ExecuteTrajectory(moveit_msgs::RobotTrajectory trajectory);
    void ExecutePoseSeries(std::vector<geometry_msgs::PoseStamped> poses);

    bool CheckTrajecotry(moveit_msgs::RobotTrajectory trajectory);

    void SetPlanningScene();

    geometry_msgs::PoseStamped GetEndEffectorPose();
    void PlanSimulationPath();
    void UpdateCurrentState();
    void MoveToValidRandom();
    bool GoHome();
    bool GoPosition(std::string positionName);
    void Execute();
    void ReplanTrajectory(moveit_msgs::RobotTrajectory& trajectory);


    //moveIt Stuff
    moveit::planning_interface::MoveGroupInterface* _moveGroup;
    moveit::planning_interface::MoveGroupInterface::Plan _currentPlan;
    moveit::planning_interface::MoveGroupInterface::Plan _currentExecutedPlan;
    void ClearConstraints();
    void SetConstraints(moveit_msgs::Constraints);

    moveit::planning_interface::PlanningSceneInterface* _planningSceneInterface;
    planning_scene::PlanningScene* _planningScene;


    visualization_msgs::MarkerArray CreateWaypointMarker(std::vector<geometry_msgs::Pose> waypoints,
                                                         std::string frameID);
    visualization_msgs::MarkerArray CreateWaypointMarker(std::vector<geometry_msgs::PoseStamped> waypoints);

    bool PublishWaypoints(std::vector<geometry_msgs::PoseStamped> waypoints);
    bool PublishWaypoints(visualization_msgs::MarkerArray marker);

private:
    //node stuff
    ros::NodeHandle* _node;
    std::string _nodeName;
    void Run();
    boost::thread* _nodeThread;

    std::string _groupName;

    helper::TransformationHandler* _transformationHandler;

    void Init(ros::NodeHandle* parentNode);


    void InitParameter();

    void RunPlanning();
    void RunPlanningExecute();
    void MoveToValidRandomRun();
    boost::thread* _planningThread;

    //publisher
    void PublishMarker();
    ros::Publisher _pubTargetPose;
    ros::Publisher _pubWaypoints;
    void InitMarker();
    visualization_msgs::Marker _markerTargetPose;



    //parameter stuff
    customparameter::ParameterHandler* _parameterHandler;
    customparameter::Parameter<int> _param_RefreshRate;
    customparameter::Parameter<float> _paramMaxVelocityScale;
    customparameter::Parameter<float> _paramMaxAccelerationScale;
    customparameter::Parameter<float> _paramPlanningTime;
    customparameter::Parameter<float> _paramSecurityRange;
};
