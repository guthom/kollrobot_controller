#include "KollrobotMoveGroup.h"
#include <boost/thread.hpp>


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
    /*

    if(_planningThread != NULL)
    {
        _planningThread = new boost::thread(boost::bind(&KollrobotMoveGroup::RunPlanning,this));
    }
     */
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

    /*

    if(_planningThread != NULL && IsExecuting == false)
    {
        _planningThread = new boost::thread(boost::bind(&KollrobotMoveGroup::RunPlanningExecute,this));
    }

    */
}

void KollrobotMoveGroup::GoHome()
{
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
    /*
    if(_planningThread != NULL && IsExecuting == false)
    {
        _planningThread = new boost::thread(boost::bind(&KollrobotMoveGroup::MoveToValidRandomRun,this));
    }
     */
}

void KollrobotMoveGroup::MoveToValidRandomRun()
{
    _moveGroup->setRandomTarget();
    IsExecuting = true;
    _moveGroup->move();
    IsExecuting = false;
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
