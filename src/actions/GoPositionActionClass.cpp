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


    void GoPositionActionClass::ExecuteActionCallback(const ActionGoal goal) {

        return;
    }


    void GoPositionActionClass::Init() {
        _server = new ActionServer(*_node, _actionName, boost::bind(&GoPositionActionClass::ExecuteActionCallback, this,
                                                                    _1), false);

        _server->start();
        ROS_INFO_STREAM("Started Action Server for " << _actionName);

    }
}