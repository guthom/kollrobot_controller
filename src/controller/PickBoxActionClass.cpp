//
// Created by thomas on 11.06.18.
//
#include "PickBoxActionClass.h"

PickBoxActionClass::PickBoxActionClass(ros::NodeHandle* node, KollrobotMoveGroup* moveGroup) : _node(node), _moveGroup(moveGroup)
{
    Init();
}

void PickBoxActionClass::PublishFeedback(std::string state, float percent)
{
    ROS_INFO_STREAM(state);
    _feedback.current_state = state;
    _feedback.percent_complete = percent;

    _server->publishFeedback(_feedback);
}

void PickBoxActionClass::ExecuteActionCallback(const kollrobot_controller::PickBoxGoalConstPtr& goal)
{
    ROS_INFO_STREAM("Startet new " << _actionName);

    bool success = true;
    auto newGoal = goal.get();

    PublishFeedback("Moving to pre pick position", 0.0);
    _moveGroup->PlanToPoseExecute(newGoal->box_pose);
    PublishFeedback("Moved to to pre pick position", 10.0);

    PublishFeedback("Calculating Gripping Trajectory", 20.0);

    PublishFeedback("Execute Gripping Trajectory", 40.0);

    PublishFeedback("Moving back to pre pick position", 90.0);
    _moveGroup->PlanToPoseExecute(newGoal->box_pose);


    PublishFeedback("Finished pick action", 100.0);
    //_result.succeed = success;
    _server->setSucceeded(_result);
    ROS_INFO_STREAM("Finished " << _actionName);
}

void PickBoxActionClass::Init()
{
    _server = new PickBoxActionServer(*_node, _actionName, boost::bind(&PickBoxActionClass::ExecuteActionCallback, this,
                                                                       _1), false);

    _server->start();
    ROS_INFO_STREAM("Started Action Server for " << _actionName);

}
