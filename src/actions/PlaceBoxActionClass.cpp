//
// Created by thomas on 11.06.18.
//
#include "PlaceBoxActionClass.h"

namespace PlaceBoxAction {

    PlaceBoxActionClass::PlaceBoxActionClass(ros::NodeHandle *node, KollrobotMoveGroup *moveGroup) :
            BaseActionClass(node, moveGroup) {
        _actionName = "PlaceBoxAction";
        Init();
    }

    void PlaceBoxActionClass::PublishFeedback(std::string state, float percent, bool warn = false) {
        if (!warn) {
            ROS_INFO_STREAM(state);
        } else {
            ROS_ERROR_STREAM(state);
        }

        _feedback.percent_complete = percent;

        _server->publishFeedback(_feedback);
    }


    void PlaceBoxActionClass::ExecuteActionCallback(const ActionGoal goal)
    {

        PublishFeedback("Start GoPositionAction!", 0.0);

        if (_moveGroup->IsBusy())
        {
            ROS_INFO_STREAM("Can not run action, movegroup is busy!");
            PublishFeedback("Can't start pick action", 100.0, true);
        }


        _result.succeed = true;
        _server->setSucceeded(_result);

        return;
    }

}