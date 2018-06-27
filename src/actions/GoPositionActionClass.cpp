//
// Created by thomas on 11.06.18.
//
#include "GoPositionActionClass.h"

namespace GoPositionAction {
    GoPositionActionClass::GoPositionActionClass(ros::NodeHandle *node, KollrobotMoveGroup *moveGroup) :
            BaseActionClass(node, moveGroup) {
        _actionName = "GoPositionAction";
        Init();
    }

    void GoPositionActionClass::PublishFeedback(std::string state, float percent, bool warn = false) {
        if (!warn) {
            ROS_INFO_STREAM(state);
        } else {
            ROS_ERROR_STREAM(state);
        }

        _feedback.percent_complete = percent;

        _server->publishFeedback(_feedback);
    }


    void GoPositionActionClass::ExecuteActionCallback(const ActionGoal goal)
    {

        PublishFeedback("Start GoPositionAction!", 0.0);

        if (_moveGroup->IsBusy())
        {
            ROS_INFO_STREAM("Can not run action, movegroup is busy!");
            PublishFeedback("Can't start pick action", 100.0, true);
        }

        std::string position = goal.get()->position;

        PublishFeedback("Plan to known position: " + position, 10.0);

        bool success = _moveGroup->GoPosition(goal.get()->position);

        if(!success)
        {
            PublishFeedback("Can not plan to given pose!", 100.0, true);
        } else
        {

            PublishFeedback("Reached Position - finished Action!", 100.0);
        }

        _result.succeed = success;
        _server->setSucceeded(_result);

        return;
    }


    void GoPositionActionClass::Init() {
        _server = new ActionServer(*_node, _actionName, boost::bind(&GoPositionActionClass::ExecuteActionCallback, this,
                                                                    _1), false);

        _server->start();
        ROS_INFO_STREAM("Started Action Server for " << _actionName);

    }
}