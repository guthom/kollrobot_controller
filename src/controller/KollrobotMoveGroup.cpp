#include "KollrobotMoveGroup.h"

KollrobotMoveGroup::KollrobotMoveGroup(ros::NodeHandle* parentNode, customparameter::ParameterHandler* parameterHandler, std::string groupName)
    : _parameterHandler(parameterHandler), _groupName(groupName)
{
    Init(parentNode);
}

void KollrobotMoveGroup::Init(ros::NodeHandle* parentNode)
{
    _nodeName = "MG_" + _groupName;
    _node = new ros::NodeHandle(*parentNode, _nodeName);


    InitParameter();

    //init MoveIT Stuff
    _moveGroup = new moveit::planning_interface::MoveGroupInterface(_groupName);
    //TODO: Velocity Hack! Add Parameter for this
    _moveGroup->setMaxVelocityScalingFactor(double(0.2));
    _moveGroup->setMaxAccelerationScalingFactor(double(0.2));
    ROS_ERROR("Reduced Speed for ROPOSE");

    _planningSzene = new moveit::planning_interface::PlanningSceneInterface();

    //init publisher
    _pubTargetPose = _node->advertise<visualization_msgs::Marker>(_node->getNamespace() + "/TargetPose", 100);
    _pubWaypoints = _node->advertise<visualization_msgs::MarkerArray>(_node->getNamespace() + "/Waypoints", 100);

    //run node
    _nodeThread = new boost::thread(boost::bind(&KollrobotMoveGroup::Run,this));

    InitMarker();
}

void KollrobotMoveGroup::InitParameter()
{
    std::string subNamespace = _nodeName + "/";
    _param_RefreshRate = _parameterHandler->AddParameter("RefreshRate", subNamespace, "", int(20));
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

void KollrobotMoveGroup::ExecuteTrajectory(moveit_msgs::RobotTrajectory trajectory)
{
    _currentPlan.trajectory_ = trajectory;
    Execute();
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

moveit_msgs::RobotTrajectory KollrobotMoveGroup::ComputeCartesianpath(std::vector<geometry_msgs::Pose> waypoints)
{
    visualization_msgs::MarkerArray waypointMarker = CreateWaypointMarker(waypoints);
    _pubWaypoints.publish(waypointMarker);

    const double jump_threshold = 0.0;
    const double eef_step = 0.005;

    moveit_msgs::RobotTrajectory trajectory;
    trajectory.joint_trajectory.header.frame_id = "simulatedQR";
    _moveGroup->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    return trajectory;
}

void KollrobotMoveGroup::PlanToPoseExecute(geometry_msgs::PoseStamped targetPose)
{
    _moveGroup->setPoseTarget(targetPose);
    _markerTargetPose.header.frame_id = targetPose.header.frame_id;
    _markerTargetPose.pose = targetPose.pose;

    RunPlanningExecute();
}

void KollrobotMoveGroup::GoHome()
{
    _moveGroup->setNamedTarget("home");
    RunPlanningExecute();
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
    _moveGroup->setRandomTarget();
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
