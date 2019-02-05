#include "KollrobotMoveGroup.h"
#include <map>
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


std::string KollrobotMoveGroup::GetSaveStartPosition(geometry_msgs::TransformStamped target)
{
    return "save_left";
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


bool KollrobotMoveGroup::CheckTollerance(float x1, float x2, float tollerance)
{
    if((x1 <= x2 + tollerance) && (x1 >= x2 - tollerance))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool KollrobotMoveGroup::CheckPosition(std::string positionName)
{
    auto knownPoses = _moveGroup->getNamedTargets();
    bool known = false;

    for(int i = 0; i < knownPoses.size(); i++)
    {
        if(knownPoses[i] == positionName)
        {
            known = true;
            break;
        }
    }

    if(!known)
    {
        ROS_WARN_STREAM("Given Pose is not known by the controller!");
        return false;
    }

    auto currentStateValues = _moveGroup->getCurrentJointValues();
    auto stateValueMap = _moveGroup->getNamedTargetValues(positionName);
    float tollerance = _paramStateCheckTollerance.GetValue();

    std::vector<std::string> linkMap = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    for (int i = 0; i < linkMap.size(); i++)
    {
        float isVal = currentStateValues[i];
        float shouldVal = stateValueMap[linkMap[i]];

        if(!CheckTollerance(shouldVal, isVal, tollerance))
        {
            return false;
        }
    }

    return true;
}

void KollrobotMoveGroup::SetPlanningScene()
{
    //create hacked scene for save planning with kollrobot
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
    float handle[3] = {0.175f + securityRange, 0.1f + securityRange/4, 0.080f};
    float camera[3] = {0.12f + securityRange, 0.05f + securityRange/4, 0.06f};
    float gripper[3] = {0.076f, 0.085f, 0.085f};

    //add groundplane
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 10.0;
    primitive.dimensions[1] = 10.0;
    primitive.dimensions[2] = 0.02;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.9;

    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(box_pose);

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

    //TODO: Get pose from transformations!
    float camZ = 0.08f;
    primitive.dimensions[0] = camera[0];
    primitive.dimensions[1] = camera[1];
    primitive.dimensions[2] = camera[2];
    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.707;
    box_pose.orientation.z = 0.0;
    box_pose.orientation.w = 0.707;
    box_pose.position.x = 0.0;
    box_pose.position.y = + 0.09f + camera[1]/2;
    box_pose.position.z = - camZ;
    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(box_pose);

    box_pose.orientation.x = 0.707;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.707;
    box_pose.orientation.w = 0.0;
    box_pose.position.x = 0.275f + camera[1]/2;
    box_pose.position.y = - 0.12f ;
    box_pose.position.z = - camZ;
    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(box_pose);

    box_pose.position.x = - 0.275f - camera[1]/2;
    box_pose.position.y = - 0.12f ;
    box_pose.position.z = - camZ;
    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(box_pose);

    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.0;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  gripper[1]/2;
    box_pose.position.y = - 0.0;
    box_pose.position.z = - 0.0;

    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(box_pose);


    //ROS_INFO("Added approx kollrobot for planning!!");
    _planningSceneInterface->applyCollisionObject(co);
}

void KollrobotMoveGroup::InitParameter()
{
    std::string subNamespace = _nodeName + "/";
    _param_RefreshRate = _parameterHandler->AddParameter("RefreshRate", subNamespace, "", int(20));
    _paramMaxAccelerationScale = _parameterHandler->AddParameter("MaxAccelerationScale", subNamespace, "", 0.1f);
    _paramMaxVelocityScale = _parameterHandler->AddParameter("MaxVelocityScale", subNamespace, "", 0.1f);
    _paramStateCheckTollerance = _parameterHandler->AddParameter("StateCheckTollerance", subNamespace, "", 0.1f);
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


void KollrobotMoveGroup::ExecutePoseSeriesAsTrajectory(std::vector<geometry_msgs::PoseStamped> poses,
                                                       std::vector<float> speeds, std::string frameID)
{
    visualization_msgs::MarkerArray waypointMarker = CreateWaypointMarker(poses);
    _pubWaypoints.publish(waypointMarker);
    IsTrajecotoryExecuting = true;
    for(int i = 0; i < poses.size(); i++)
    {
        auto traj = CalculateTrajectory(poses[i], speeds[i]);
        ExecuteTrajectory(traj, frameID);
    }
    IsTrajecotoryExecuting = false;
}

void KollrobotMoveGroup::ExecuteTrajectory(moveit_msgs::RobotTrajectory trajectory)
{

    IsTrajecotoryExecuting = true;
    _currentPlan.trajectory_ = trajectory;
    Execute();

    IsTrajecotoryExecuting = false;
}


void KollrobotMoveGroup::ExecuteTrajectory(robot_trajectory::RobotTrajectory traj, std::string frameID)
{
    IsTrajecotoryExecuting = true;
    _currentPlan.trajectory_ = ToTrajectoryMSG(traj, frameID);
    Execute();
    IsTrajecotoryExecuting = false;
}

void KollrobotMoveGroup::ExecuteTrajectory(std::vector<robot_trajectory::RobotTrajectory> trajectory,
                                           std::string frameID)
{
    IsTrajecotoryExecuting = true;
    for(int i = 0; i < trajectory.size(); i++)
    {
        _currentPlan.trajectory_ = ToTrajectoryMSG(trajectory[i], frameID);
        Execute();
    }
    IsTrajecotoryExecuting = false;
}

void KollrobotMoveGroup::ExecuteTrajectory(std::vector<moveit_msgs::RobotTrajectory> trajectory)
{
    IsTrajecotoryExecuting = true;
    for(int i = 0; i < trajectory.size(); i++)
    {
        _currentPlan.trajectory_ = trajectory[i];
        Execute();
    }
    IsTrajecotoryExecuting = false;
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


robot_trajectory::RobotTrajectory KollrobotMoveGroup::CalculateTrajectory(geometry_msgs::PoseStamped pose,
                                                                          float speed=1.0)
{

    //arrange waypoints also for the use in EIGEN
    std::vector<geometry_msgs::Pose> waypoints;
    EigenSTL::vector_Affine3d  eigen_waypoints;
    Eigen::Affine3d point;
    tf::poseMsgToEigen(pose.pose, point);
    eigen_waypoints.push_back(point);

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


    double fraction = startState.computeCartesianPath(jmg, lastTraj, startState.getLinkModel(linkName), eigen_waypoints,
                                                      true, max_step, jump_threshold);
    ROS_INFO_STREAM("Calculated Trajectory with a fraction of " + std::to_string(fraction));
    robot_trajectory::RobotTrajectory rt(robotModel, _groupName);

    for (int i = 0; i < lastTraj.size(); ++i)
        rt.addSuffixWayPoint(lastTraj[i], 0.0);

    //retime trajectory with timing given by the speed vector and the max scalling ros-parameter
    time_param.computeTimeStamps(rt, velocityScale*speed, accelScale*speed);

    return rt;
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
        moveit_msgs::RobotTrajectory trajectory = ToTrajectoryMSG(trajectories[i], frameID);
    }

    return ret;
}

moveit_msgs::RobotTrajectory
KollrobotMoveGroup::ToTrajectoryMSG(robot_trajectory::RobotTrajectory trajectory,
                                                          std::string frameID)
{
    moveit_msgs::RobotTrajectory traj;
    trajectory.getRobotTrajectoryMsg(traj);

    traj.joint_trajectory.header.frame_id = frameID;
    traj.multi_dof_joint_trajectory.header.frame_id = frameID;

    return traj;
}

robot_trajectory::RobotTrajectory
KollrobotMoveGroup::CalculateTrajectory(std::vector<geometry_msgs::PoseStamped> poseSeries)
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


    double fraction = startState.computeCartesianPath(jmg, lastTraj, startState.getLinkModel(linkName), eigen_waypoints,
                                                          true, max_step, jump_threshold);
    ROS_INFO_STREAM("Calculated Trajectory with a fraction of " + std::to_string(fraction));
    robot_trajectory::RobotTrajectory rt(robotModel, _groupName);

    for (int i = 0; i < lastTraj.size(); ++i)
        rt.addSuffixWayPoint(lastTraj[i], 0.0);

        //retime trajectory with timing given by the speed vector and the max scalling ros-parameter
    time_param.computeTimeStamps(rt, velocityScale, accelScale);

    return rt;
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
            startState = *lastTraj[lastTraj.size()-1];
        }

        double fraction = startState.computeCartesianPath(jmg, lastTraj, startState.getLinkModel(linkName), eigen_waypoints[i],
                                                          true, max_step, jump_threshold);
        ROS_INFO_STREAM("Calculated Trajectory with a fraction of " + std::to_string(fraction));
        robot_trajectory::RobotTrajectory rt(robotModel, _groupName);
        for (int i = 0; i < lastTraj.size(); ++i)
            rt.addSuffixWayPoint(lastTraj[i], 0.0);

        //retime trajectory with timing given by the speed vector and the max scalling ros-parameter
        time_param.computeTimeStamps(rt, speeds[i]*velocityScale, speeds[i]*accelScale);

        trajectories.push_back(rt);
    }
    return trajectories;

}

robot_trajectory::RobotTrajectory
KollrobotMoveGroup::FuseTrajectories(std::vector<robot_trajectory::RobotTrajectory> trajectories)
{
    robot_trajectory::RobotTrajectory _rt(_moveGroup->getRobotModel(), _groupName);
    for(int i = 0; i < trajectories.size(); i++)
    {

        _rt.append(trajectories[i], 0.0);
    }

    return _rt;
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
    const std::string groupName = _groupName;
    for(int i = 0; i<trajectory.size(); i++)
    {

        bool valid = CheckTrajecotry(trajectory[i]);

        if(!valid)
        {
            return true;
        }
    }

    return true;
}

bool KollrobotMoveGroup::CheckTrajecotry(robot_trajectory::RobotTrajectory trajectory)
{
    moveit_msgs::Constraints constraints = _moveGroup->getPathConstraints();

    const std::string groupName = _groupName;

    bool valid = _planningScene->isPathValid(trajectory, constraints, groupName);

    return valid;
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
