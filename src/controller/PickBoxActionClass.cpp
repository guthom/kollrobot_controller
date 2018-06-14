//
// Created by thomas on 11.06.18.
//
#include "PickBoxActionClass.h"

PickBoxActionClass::PickBoxActionClass(ros::NodeHandle* node,  KollrobotMoveGroup* moveGroup) : _node(node),
                                                                                                _moveGroup(moveGroup)
{
    Init();
}

void PickBoxActionClass::PublishFeedback(std::string state, float percent, bool warn=false)
{
    if(!warn)
    {
        ROS_INFO_STREAM(state);
    }
    else
    {
        ROS_ERROR_STREAM(state);
    }

    _feedback.current_state = state;
    _feedback.percent_complete = percent;

    _server->publishFeedback(_feedback);
}


geometry_msgs::PoseStamped PickBoxActionClass::CalculatePrePickPosition(geometry_msgs::PoseStamped targetPose)
{
    geometry_msgs::PoseStamped prePickPose(targetPose);
    prePickPose.pose.position.z += 0.1;
    prePickPose.pose.position.x -= 0.05;
    //force quaternion
    //TODO: Find a better way to to this
    //prePickPose.pose.orientation.x = 1.0;
    //prePickPose.pose.orientation.y = 0.0;
    //prePickPose.pose.orientation.z = 0.0;
    //prePickPose.pose.orientation.w = 0.0;

    return prePickPose;
}

moveit_msgs::RobotTrajectory PickBoxActionClass::CalculatePickTrajectory(geometry_msgs::PoseStamped prePickPose)
{
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(prePickPose.pose);

    geometry_msgs::Pose pose1 = waypoints[0];
    pose1.position.z = 0.0;
    waypoints.push_back(pose1);

    geometry_msgs::Pose pose2 = waypoints[1];
    pose2.position.x = 0.0;
    waypoints.push_back(pose2);

    moveit_msgs::RobotTrajectory trajecotry = _moveGroup->ComputeCartesianpath(waypoints);
    trajecotry.joint_trajectory.header.frame_id = prePickPose.header.frame_id;

    return trajecotry;
}


bool PickBoxActionClass::CheckBoxAvailability(std::string boxFrameID)
{
    return _transformationHandler.FrameExist(boxFrameID);
}

void PickBoxActionClass::ExecuteActionCallback(const kollrobot_controller::PickBoxGoalConstPtr& goal)
{
    ROS_INFO_STREAM("Startet new " << _actionName);


    bool success = true;
    if (_moveGroup->IsBusy())
    {
        PublishFeedback("MoveGroup is busy! Can't start action!", 0.0);
        success = false;
        _server->setSucceeded(_result);
        return;
    }

    auto newGoal = goal.get();

    if(!CheckBoxAvailability(newGoal->box_frameID))
    {
        std::string msg = "Box Transformation - " + newGoal->box_frameID + " - is not available, can't pick it!";
        PublishFeedback(msg, 0.0);
        success = false;
        _server->setSucceeded(_result);
        return;
    }

    PublishFeedback("Moving to pre pick position", 0.0);
    auto prePickPose = CalculatePrePickPosition(newGoal->box_pose);
    _moveGroup->PlanToPoseExecute(prePickPose);
    PublishFeedback("Moved to to pre pick position", 10.0);

    PublishFeedback("Calculating Gripping Trajectory", 20.0);
    moveit_msgs::RobotTrajectory trajectory = CalculatePickTrajectory(prePickPose);

    PublishFeedback("Execute Gripping Trajectory", 40.0);
    _moveGroup->ExecuteTrajectory(trajectory);


    PublishFeedback("Moving back to pre pick position", 80.0);
    _moveGroup->PlanToPoseExecute(prePickPose);

    PublishFeedback("Moving back to home position position", 90.0);
    _moveGroup->GoHome();

    PublishFeedback("Finished pick action", 100.0);
    //_result.succeed = success;
    _server->setSucceeded(_result);
    return;
}

void PickBoxActionClass::Init()
{
    _server = new PickBoxActionServer(*_node, _actionName, boost::bind(&PickBoxActionClass::ExecuteActionCallback, this,
                                                                       _1), false);

    _server->start();
    ROS_INFO_STREAM("Started Action Server for " << _actionName);

}
