#include "KollrobotMoveGroup.h"
#include <boost/thread.hpp>


KollrobotMoveGroup::KollrobotMoveGroup(ros::NodeHandle* parentNode, customparameter::ParameterHandler* parameterHandler, std::string groupName)
    : _parameterHandler(parameterHandler), _groupName(groupName)
{
    Init(parentNode);
}

void KollrobotMoveGroup::Init(ros::NodeHandle* parentNode)
{
    _nodeName = "MoveGroup_" + _groupName;
    _node = new ros::NodeHandle(*parentNode, _nodeName);

    //init MoveIT Stuff
    _moveGroup = new moveit::planning_interface::MoveGroupInterface(_groupName);
    _planningSzene = new moveit::planning_interface::PlanningSceneInterface();

    //init publisher
    _pubTargetPose = _node->advertise<visualization_msgs::Marker>(_node->getNamespace() + "/TargetPose", 100);

    //run node
    _nodeThread = new boost::thread(boost::bind(&KollrobotMoveGroup::Run,this));

}

void KollrobotMoveGroup::InitMarker()
{
    _markerTargetPose = visualization_msgs::Marker();
    //TODO: add aprameter to change refernce frame
    _markerTargetPose.header.frame_id =  "world";
    _markerTargetPose.type = visualization_msgs::Marker::SPHERE;
    _markerTargetPose.action = visualization_msgs::Marker::ADD;
    _markerTargetPose.scale.x = 0.10;
    _markerTargetPose.scale.y = 0.10;
    _markerTargetPose.scale.z = 0.10;
    _markerTargetPose.color.a = 1.0;
    _markerTargetPose.color.r = 0.0;
    _markerTargetPose.color.g = 1.0;
    _markerTargetPose.color.b = 0.0;
}

void KollrobotMoveGroup::PlanToPose(geometry_msgs::Pose targetPose)
{
    _moveGroup->setPoseTarget(targetPose);
    _markerTargetPose.pose = targetPose;

    if(_planningThread != NULL)
    {
        _planningThread = new boost::thread(boost::bind(&KollrobotMoveGroup::RunPlanning,this));
    }
}

void KollrobotMoveGroup::PlanToPoseExecute(geometry_msgs::Pose targetPose)
{
    _moveGroup->setPoseTarget(targetPose);
    _markerTargetPose.pose = targetPose;

    if(_planningThread != NULL)
    {
        _planningThread = new boost::thread(boost::bind(&KollrobotMoveGroup::RunPlanningExecute,this));
    }
}

void KollrobotMoveGroup::Run()
{
    ros::Rate rate(_refreshRate);
    while(_node->ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void KollrobotMoveGroup::Execute()
{
    _currentExecutedPlan = _currentPlan;
    _moveGroup->execute(_currentExecutedPlan);
}

void KollrobotMoveGroup::RunPlanning()
{
    IsPlanning = true;

    PlanValid = _moveGroup->plan(_currentPlan);


    IsPlanning = false;
}


void KollrobotMoveGroup::RunPlanningExecute()
{
    IsPlanning = true;
    PlanValid = _moveGroup->plan(_currentPlan);
    IsPlanning = false;
    Execute();
}

KollrobotMoveGroup::~KollrobotMoveGroup()
{
    //release everithing and set pointer to NULL
    _parameterHandler = NULL;
}
