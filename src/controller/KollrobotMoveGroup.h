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

    std::string _groupName;
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

    std::vector<robot_trajectory::RobotTrajectory> CalculateTrajectory(std::vector<geometry_msgs::PoseStamped> poseSeries,
                                                                  std::vector<float> speeds);

    robot_trajectory::RobotTrajectory CalculateTrajectory(std::vector<geometry_msgs::PoseStamped> poseSeries);

    robot_trajectory::RobotTrajectory CalculateTrajectory(geometry_msgs::PoseStamped pose, float speed);

    std::vector<robot_trajectory::RobotTrajectory> CalculateTrajectory(std::vector<geometry_msgs::PoseStamped> poseSeries,
                                                                       std::vector<float> speeds,
                                                                       geometry_msgs::TransformStamped transform);

    robot_trajectory::RobotTrajectory FuseTrajectories(std::vector<robot_trajectory::RobotTrajectory> trajectories);

    std::vector<moveit_msgs::RobotTrajectory> ToTrajectoryMSG(std::vector<robot_trajectory::RobotTrajectory> trajectories,
                                                                                  std::string frameID);

    moveit_msgs::RobotTrajectory ToTrajectoryMSG(robot_trajectory::RobotTrajectory trajectories,
                                                              std::string frameID);

    void ExecuteTrajectory(moveit_msgs::RobotTrajectory trajectory);
    void ExecuteTrajectory(std::vector<moveit_msgs::RobotTrajectory>);
    void ExecuteTrajectory(std::vector<robot_trajectory::RobotTrajectory> traj, std::string frameID);
    void ExecuteTrajectory(robot_trajectory::RobotTrajectory traj, std::string frameID);

    void ExecutePoseSeries(std::vector<geometry_msgs::PoseStamped> poses);
    void ExecutePoseSeriesAsTrajectory(std::vector<geometry_msgs::PoseStamped> poses, std::vector<float> speeds,
                                       std::string frameID);

    bool CheckTrajecotry(std::vector<robot_trajectory::RobotTrajectory> trajectory);
    bool CheckTrajecotry(robot_trajectory::RobotTrajectory trajectory);

    void SetPlanningScene();

    geometry_msgs::PoseStamped GetEndEffectorPose();

    std::string GetSaveStartPosition( geometry_msgs::TransformStamped target);

    bool CheckPosition(std::string positionName);
    bool IsTrajecotoryExecuting = false;

    void PlanSimulationPath();
    void UpdateCurrentState();
    void MoveToValidRandom();
    bool GoHome();
    bool GoPosition(std::string positionName);
    void Execute();

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


    helper::TransformationHandler* _transformationHandler;

    void Init(ros::NodeHandle* parentNode);


    void InitParameter();

    bool CheckTollerance(float x1, float x2, float tollerance);

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
    customparameter::Parameter<float> _paramStateCheckTollerance;
    customparameter::Parameter<float> _paramMaxAccelerationScale;
    customparameter::Parameter<float> _paramPlanningTime;
    customparameter::Parameter<float> _paramSecurityRange;
};
