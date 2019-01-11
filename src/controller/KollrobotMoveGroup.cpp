#include "KollrobotMoveGroup.h"

#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/collision_detection/collision_tools.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>


KollrobotMoveGroup::KollrobotMoveGroup(ros::NodeHandle* parentNode, customparameter::ParameterHandler* parameterHandler, std::string groupName)
    : _parameterHandler(parameterHandler), _groupName(groupName)
{
    Init(parentNode);
}

void KollrobotMoveGroup::Init(ros::NodeHandle* parentNode)
{
    _nodeName = "MG_" + _groupName;
    _node = new ros::NodeHandle(*parentNode, _nodeName);

    _transformationHandler = new helper::TransformationHandler(_node);

    InitParameter();

    //init MoveIT Stuff
    _moveGroup = new moveit::planning_interface::MoveGroupInterface(_groupName);
    _moveGroup->setStartStateToCurrentState();

    _moveGroup->setMaxVelocityScalingFactor(double(_paramMaxVelocityScale.GetValue()));
    _moveGroup->setMaxAccelerationScalingFactor(double(_paramMaxAccelerationScale.GetValue()));
    _moveGroup->setPlanningTime(_paramPlanningTime.GetValue());

    _planningSceneInterface = new moveit::planning_interface::PlanningSceneInterface();
    _planningScene = new planning_scene::PlanningScene(_moveGroup->getRobotModel());

    //init publisher
    _pubTargetPose = _node->advertise<visualization_msgs::Marker>(_node->getNamespace() + "/TargetPose", 100);
    _pubWaypoints = _node->advertise<visualization_msgs::MarkerArray>(_node->getNamespace() + "/Waypoints", 100);

    //run node
    _nodeThread = new boost::thread(boost::bind(&KollrobotMoveGroup::Run,this));


    //GoHome();
    SetPlanningScene();

    InitMarker();
}

geometry_msgs::PoseStamped KollrobotMoveGroup::GetEndEffectorPose()
{
    return _moveGroup->getCurrentPose(_moveGroup->getEndEffectorLink());
}

void KollrobotMoveGroup::SetConstraints(moveit_msgs::Constraints constraints)
{
    ROS_INFO_STREAM("Set constraints for path planning!");
    _moveGroup->setPathConstraints(constraints);
}

void KollrobotMoveGroup::ClearConstraints()
{
    ROS_INFO_STREAM("Cleared all constraints for path planning!");
    _moveGroup->clearPathConstraints();

}

void KollrobotMoveGroup::SetPlanningScene()
{
    //creat hacked scene for save planning with kollrobot
    moveit_msgs::CollisionObject co;
    co.id = "kollrobotApprox";
    co.operation = co.ADD;
    shape_msgs::SolidPrimitive primitive;

    //add first primitive box
    primitive.type = primitive.BOX;
    geometry_msgs::Pose box_pose;

    float securityRange = _paramSecurityRange.GetValue();

    float robOffset[3] = {0.0f, -0.235f, 0.0f};
    float bigBox[3] = {0.56f + securityRange, 0.65f + securityRange, 0.90f};
    float smallBox[3] = {0.56f + securityRange, 0.25f + securityRange/4, 0.095f};
    float handle[3] = {0.175f + securityRange, 0.03f + securityRange/4, 0.045f};


    primitive.dimensions.resize(3);
    primitive.dimensions[0] = bigBox[0];
    primitive.dimensions[1] = bigBox[1];
    primitive.dimensions[2] = bigBox[2];
    box_pose.orientation.w = 1.0;
    box_pose.position.x = robOffset[0];
    box_pose.position.y = robOffset[1];
    box_pose.position.z = robOffset[2] - bigBox[2]/2;

    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(box_pose);

    //add second primitive box
    primitive.dimensions[0] = smallBox[0];
    primitive.dimensions[1] = smallBox[1];
    primitive.dimensions[2] = smallBox[2];
    box_pose.orientation.w = 1.0;
    box_pose.position.x = robOffset[0];
    box_pose.position.y = -(bigBox[1]/2 - smallBox[1]/2 - securityRange/2 - robOffset[1]);
    box_pose.position.z = robOffset[2] + smallBox[2]/2;

    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(box_pose);

    //add handle primitive box
    primitive.dimensions[0] = handle[0];
    primitive.dimensions[1] = handle[1];
    primitive.dimensions[2] = handle[2];
    box_pose.orientation.w = 1.0;
    box_pose.position.x += 0.025;
    box_pose.position.z += smallBox[2]/2 + handle[2]/2;

    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(box_pose);

    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(box_pose);


    //ROS_INFO("Added approx kollrobot for planning!!");
    _planningSceneInterface->applyCollisionObject(co);
}

void KollrobotMoveGroup::InitParameter()
{
    std::string subNamespace = _nodeName + "/";
    _param_RefreshRate = _parameterHandler->AddParameter("RefreshRate", subNamespace, "", int(20));
    _paramMaxAccelerationScale = _parameterHandler->AddParameter("MaxAccelerationScale", subNamespace, "", 1.0f);
    _paramMaxVelocityScale = _parameterHandler->AddParameter("MaxVelocityScale", subNamespace, "", 1.0f);
    _paramPlanningTime = _parameterHandler->AddParameter("PlanningTime", subNamespace, "", 5.0f);
    _paramSecurityRange = _parameterHandler->AddParameter("SecurityRange", subNamespace, "", 0.02f);
}

void KollrobotMoveGroup::InitMarker()
{
    _markerTargetPose = visualization_msgs::Marker();
    _markerTargetPose.header.frame_id = "base_link";
    _markerTargetPose.type = visualization_msgs::Marker::SPHERE;
    _markerTargetPose.action = visualization_msgs::Marker::ADD;
    _markerTargetPose.scale.x = 0.05;
    _markerTargetPose.scale.y = 0.05;
    _markerTargetPose.scale.z = 0.05;
    _markerTargetPose.color.a = 1.0;
    _markerTargetPose.color.r = 0.0;
    _markerTargetPose.color.g = 1.0;
    _markerTargetPose.color.b = 0.0;
}

void KollrobotMoveGroup::PlanToPose(geometry_msgs::Pose targetPose)
{
    _moveGroup->setPoseTarget(targetPose);
    _markerTargetPose.pose = targetPose;

    RunPlanning();
}

void KollrobotMoveGroup::PlanToPose(geometry_msgs::PoseStamped targetPose)
{
    _moveGroup->setPoseTarget(targetPose);
    _markerTargetPose.pose = targetPose.pose;
    _markerTargetPose.header.frame_id = targetPose.header.frame_id;

    RunPlanning();
}

void KollrobotMoveGroup::PublishMarker()
{
    //TODO: add aprameter to change refernce frame
    _markerTargetPose.header.stamp = ros::Time::now();
    _pubTargetPose.publish(_markerTargetPose);
}

void KollrobotMoveGroup::PlanToPoseExecute(geometry_msgs::Pose targetPose)
{
    _moveGroup->setPoseTarget(targetPose);
    _markerTargetPose.pose = targetPose;
    RunPlanningExecute();
}


void KollrobotMoveGroup::PlanToPositionExecute(geometry_msgs::PointStamped targetPose)
{
    _moveGroup->clearPoseTargets();
    _moveGroup->setPositionTarget(targetPose.point.x,targetPose.point.y,targetPose.point.z);
    _markerTargetPose.pose.position = targetPose.point;
    RunPlanningExecute();
}

void KollrobotMoveGroup::PlanToPositionExecute(geometry_msgs::Point targetPose)
{
    _moveGroup->clearPoseTargets();
    _moveGroup->setPositionTarget(targetPose.x,targetPose.y,targetPose.z);
    _markerTargetPose.pose.position = targetPose;
    RunPlanningExecute();
}

void KollrobotMoveGroup::ExecuteTrajectory(moveit_msgs::RobotTrajectory trajectory)
{
    _currentPlan.trajectory_ = trajectory;
    Execute();
}

void KollrobotMoveGroup::ExecuteTrajectory(std::vector<moveit_msgs::RobotTrajectory> trajectory)
{
    for(int i = 0; i < trajectory.size(); i++)
    {
        _currentPlan.trajectory_ = trajectory[i];
        Execute();
    }
}


bool KollrobotMoveGroup::PublishWaypoints(visualization_msgs::MarkerArray marker)
{
    _pubWaypoints.publish(marker);
    return true;
}

bool KollrobotMoveGroup::PublishWaypoints(std::vector<geometry_msgs::PoseStamped> waypoints)
{
    visualization_msgs::MarkerArray marker = CreateWaypointMarker(waypoints);
    return PublishWaypoints(marker);
}

void KollrobotMoveGroup::ExecutePoseSeries(std::vector<geometry_msgs::PoseStamped> poses)
{
    visualization_msgs::MarkerArray waypointMarker = CreateWaypointMarker(poses);
    _pubWaypoints.publish(waypointMarker);

    for(int i = 0; i < poses.size(); i++)
    {
        PlanToPose(poses[i]);
        Execute();
    }

}


visualization_msgs::MarkerArray KollrobotMoveGroup::CreateWaypointMarker(std::vector<geometry_msgs::Pose> waypoints,
                                                                         std::string frameID="base_link")
{
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker baseMarker;
    baseMarker.id = 0;
    baseMarker.header.frame_id = frameID;
    baseMarker.type = visualization_msgs::Marker::SPHERE;
    baseMarker.scale.x = 0.03;
    baseMarker.scale.y = 0.03;
    baseMarker.scale.z = 0.03;
    baseMarker.color.a = 1.0;
    baseMarker.color.r = 0.0;
    baseMarker.color.g = 0.0;
    baseMarker.color.b = 1.0;

    for(int i = 0; i < waypoints.size(); i++)
    {
        baseMarker.id += 1;
        baseMarker.pose = waypoints[i];
        markerArray.markers.push_back(baseMarker);
    }

    return markerArray;
}

visualization_msgs::MarkerArray KollrobotMoveGroup::CreateWaypointMarker(
        std::vector<geometry_msgs::PoseStamped> waypoints)
{
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker baseMarker;
    baseMarker.id = 0;
    baseMarker.type = visualization_msgs::Marker::SPHERE;
    baseMarker.scale.x = 0.03;
    baseMarker.scale.y = 0.03;
    baseMarker.scale.z = 0.03;
    baseMarker.color.a = 1.0;
    baseMarker.color.r = 0.0;
    baseMarker.color.g = 0.0;
    baseMarker.color.b = 1.0;

    for(int i = 0; i < waypoints.size(); i++)
    {
        baseMarker.id += 1;
        baseMarker.pose = waypoints[i].pose;
        baseMarker.header.frame_id = waypoints[i].header.frame_id;
        markerArray.markers.push_back(baseMarker);
    }

    return markerArray;
}

std::vector<moveit_msgs::RobotTrajectory>
KollrobotMoveGroup::ToTrajectoryMSG(std::vector<robot_trajectory::RobotTrajectory> trajectories,
                                    std::string frameID = "/world")
{
    std::vector<moveit_msgs::RobotTrajectory> ret;

    for(int i = 0; i < trajectories.size(); i++)
    {
        moveit_msgs::RobotTrajectory trajectory;
        trajectories[i].getRobotTrajectoryMsg(trajectory);

        trajectory.joint_trajectory.header.frame_id = frameID;
        trajectory.multi_dof_joint_trajectory.header.frame_id = frameID;
        ret.push_back(trajectory);
    }

    return ret;
}

std::vector<robot_trajectory::RobotTrajectory>
KollrobotMoveGroup::CalculateTrajectory(std::vector<geometry_msgs::PoseStamped> poseSeries, std::vector<float> speeds)
{
    visualization_msgs::MarkerArray waypointMarker = CreateWaypointMarker(poseSeries);
    _pubWaypoints.publish(waypointMarker);
    // Based on the the original cartesian_path_service_capability.cpp
    // But we want no service and better speed-controll
    // https://github.com/ros-planning/moveit/blob/kinetic-devel/moveit_ros/
    // move_group/src/default_capabilities/cartesian_path_service_capability.cpp

    //arrange waypoints also for the use in EIGEN
    std::vector<geometry_msgs::Pose> waypoints;
    EigenSTL::vector_Affine3d  eigen_waypoints;
    for (int i = 0; i < poseSeries.size(); i++) {
        waypoints.push_back(poseSeries[i].pose);
        Eigen::Affine3d point;
        tf::poseMsgToEigen(poseSeries[i].pose, point);
        eigen_waypoints.push_back(point);
        speeds.push_back(0.1);
    }

    robot_model::RobotModelConstPtr robotModel = _moveGroup->getRobotModel();


    robot_state::RobotState startState = *_moveGroup->getCurrentState();

    const robot_model::JointModelGroup* jmg = startState.getJointModelGroup(_groupName);

    std::string linkName = "ee_link"; //jmg->getLinkModelNames().back();

    double jump_threshold = 0.0;
    double max_step = 0.005;
    kinematics::KinematicsQueryOptions options();
    kinematic_constraints::KinematicConstraintSet kset(robotModel);

    std::vector<robot_trajectory::RobotTrajectory> trajectories;
    std::vector<robot_state::RobotStatePtr> lastTraj;

    float velocityScale = _paramMaxVelocityScale.GetValue();
    float accelScale = _paramMaxAccelerationScale.GetValue();
    trajectory_processing::IterativeParabolicTimeParameterization time_param;

    for(int i = 0; i < eigen_waypoints.size(); i++)
    {
        if(i > 0)
        {
            startState = *lastTraj[lastTraj.size()-1].get();
        }

        double fraction = startState.computeCartesianPath(jmg, lastTraj, startState.getLinkModel(linkName), eigen_waypoints[i],
                                                          true, max_step, jump_threshold);
        ROS_INFO_STREAM("Calculated Trajectory with a fraction of " + std::to_string(fraction));
        robot_trajectory::RobotTrajectory rt(robotModel, _groupName);
        for (std::size_t i = 0; i < lastTraj.size(); ++i)
            rt.addSuffixWayPoint(lastTraj[i], 0.0);

        //retime trajectory with timing given by the speed vector and the max scalling ros-parameter
        time_param.computeTimeStamps(rt, speeds[i]*velocityScale, speeds[i]*accelScale);

        trajectories.push_back(rt);
    }

    return trajectories;
}


std::vector<robot_trajectory::RobotTrajectory>
KollrobotMoveGroup::CalculateTrajectory(std::vector<geometry_msgs::PoseStamped> poseSeries, std::vector<float> speeds,
                                        geometry_msgs::TransformStamped transform)
{

    for (int i = 0; i < poseSeries.size(); i++)
    {
        poseSeries[i].pose = _transformationHandler->TransformPose(transform, poseSeries[i].pose);
        poseSeries[i].header.frame_id = transform.header.frame_id;
    }

    return CalculateTrajectory(poseSeries, speeds);
}


void KollrobotMoveGroup::PlanToPoseExecute(geometry_msgs::PoseStamped targetPose)
{
    _moveGroup->setPoseTarget(targetPose);
    _markerTargetPose.header.frame_id = targetPose.header.frame_id;
    _markerTargetPose.pose = targetPose.pose;

    RunPlanningExecute();
}

bool KollrobotMoveGroup::GoHome()
{
    return GoPosition("home");
}


bool KollrobotMoveGroup::GoPosition(std::string positionName)
{
    bool poseValid = _moveGroup->setNamedTarget(positionName);
    if(!poseValid)
    {
        ROS_WARN_STREAM("The movegroup does not know the pose: " + positionName + " Add it to the move group config");
        return false;
    }

    RunPlanningExecute();
}

bool KollrobotMoveGroup::CheckTrajecotry(std::vector<robot_trajectory::RobotTrajectory> trajectory)
{
    moveit_msgs::Constraints constraints = _moveGroup->getPathConstraints();

    const std::string groupName = _groupName;
    for(int i = 0; i<trajectory.size(); i++)
    {

        bool valid = _planningScene->isPathValid(trajectory[0], constraints, groupName);

        if(!valid)
        {
            return false;
        }
    }

    return true;
}

void KollrobotMoveGroup::Run()
{
    ros::Rate rate(_param_RefreshRate.GetValue());
    while(_node->ok())
    {
        PublishMarker();

        ros::spinOnce();
        rate.sleep();
    }
}

void KollrobotMoveGroup::MoveToValidRandom()
{
    MoveToValidRandomRun();
}

void KollrobotMoveGroup::MoveToValidRandomRun()
{
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = _groupName;
    collision_detection::CollisionResult collision_result;
    robot_state::RobotState currentState = _planningScene->getCurrentState();
    do
    {
        currentState = _planningScene->getCurrentState();
        do
        {
            collision_result.clear();
            currentState.setToRandomPositions();
            _planningScene->checkSelfCollision(collision_request, collision_result, currentState);
            ROS_INFO_STREAM("Collision Check for self Collision pose = " << collision_result.collision);
            if (collision_result.collision == false)
            {
                _planningScene->checkCollision(collision_request, collision_result, currentState);
                ROS_INFO_STREAM("Collision Check for collision pose = " << collision_result.collision);
            }

        }while (collision_result.collision);

        collision_result.clear();
        _planningScene->checkCollision(collision_request, collision_result, currentState);
        ROS_INFO_STREAM("Collision Check for random pose = " << collision_result.collision);
    }
    while (collision_result.collision);

    _moveGroup->setJointValueTarget(currentState);

    //_moveGroup->setRandomTarget();

    IsExecuting = true;
    _moveGroup->move();
    IsExecuting = false;
}

bool KollrobotMoveGroup::IsBusy()
{
    return IsExecuting || IsPlanning;
}

void KollrobotMoveGroup::Execute()
{
    _currentExecutedPlan = _currentPlan;
    _moveGroup->execute(_currentExecutedPlan);
}

void KollrobotMoveGroup::RunPlanning()
{
    IsPlanning = true;
    auto success = _moveGroup->plan(_currentPlan);
    IsPlanning = false;
}


void KollrobotMoveGroup::RunPlanningExecute()
{
    RunPlanning();
    Execute();
}

void KollrobotMoveGroup::UpdateCurrentState()
{
    _moveGroup->setStartStateToCurrentState();
}

void KollrobotMoveGroup::PlanSimulationPath()
{
//TODO: Implement if neccessary
}

KollrobotMoveGroup::~KollrobotMoveGroup()
{
    //release everything and set pointer to NULL
    _parameterHandler = NULL;
}
