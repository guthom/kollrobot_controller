//
// Created by thomas on 14.06.18.
//

#include "TransformationHandler.h"


TransformationHandler::TransformationHandler(ros::NodeHandle* parentNode, int refreshRate) :
        _iRefreshRate(refreshRate)
{
    _node = new ros::NodeHandle(*parentNode, _nodeName);

    Init();
    RunThread();
}

void TransformationHandler::RunThread()
{
    //run node
    _nodeThread = new boost::thread(boost::bind(&TransformationHandler::Run, this));
}

void TransformationHandler::Run()
{
    ros::Rate rate(_iRefreshRate);
    while(_node->ok())
    {
        sigNodeCircle();
        ros::spinOnce();
        rate.sleep();
    }
}

void TransformationHandler::Init()
{
    _tfListener = new tf2_ros::TransformListener(_tfBuffer);
    _tfBroadcaster = new tf2_ros::TransformBroadcaster();
    _tfStaticTransformBroadcaster = new tf2_ros::StaticTransformBroadcaster();
}

geometry_msgs::Transform TransformationHandler::PoseToTransform(geometry_msgs::Pose pose)
{

    //create new transform and set header and frame ids
    geometry_msgs::Transform ret;
    ret.translation.x = pose.position.x;
    ret.translation.y = pose.position.y;
    ret.translation.z = pose.position.z;
    ret.rotation = pose.orientation;

    return ret;
}

geometry_msgs::TransformStamped TransformationHandler::TransformToStamped(geometry_msgs::Transform transform,
                                                                          std::string fromFrame,
                                                                          std::string toFrame)
{
    geometry_msgs::TransformStamped ret;
    ret.transform = transform;
    ret.header.frame_id = fromFrame;
    ret.child_frame_id = toFrame;
    return ret;
}

geometry_msgs::TransformStamped TransformationHandler::PoseToTransformStamped(geometry_msgs::Pose pose,
                                                                              std::string fromFrame,
                                                                              std::string toFrame)
{
    geometry_msgs::Transform transform = PoseToTransform(pose);
    return TransformToStamped(transform, fromFrame, toFrame);
}

bool TransformationHandler::SendTransform(geometry_msgs::TransformStamped transform)
{
    bool ret = true;

    try
    {
        _tfBroadcaster->sendTransform(transform);
        ROS_INFO_STREAM("Send new transform from " << transform.header.frame_id << " to " << transform.child_frame_id);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN_STREAM("Can't send new transform" << transform.child_frame_id << ": " << ex.what());
        ret = false;
    }
    return ret;
}

bool TransformationHandler::SendTransform(geometry_msgs::Transform transform, std::string fromFrame,
                                          std::string toFrame)
{
    geometry_msgs::TransformStamped toSend = TransformToStamped(transform, fromFrame, toFrame);
    return SendTransform(toSend);
}

bool TransformationHandler::SendTransform(geometry_msgs::PoseStamped transformPose, std::string toFrame)
{
    geometry_msgs::TransformStamped toSend = PoseToTransformStamped(transformPose.pose, transformPose.header.frame_id,
                                                                    toFrame);
    return SendTransform(toSend);
}

bool TransformationHandler::SendTransform(geometry_msgs::Pose pose, std::string fromFrame, std::string toFrame)
{
    geometry_msgs::TransformStamped toSend = PoseToTransformStamped(pose, fromFrame, toFrame);
    return SendTransform(toSend);
}

bool TransformationHandler::SendStaticTransform(geometry_msgs::TransformStamped transform)
{
    bool ret = true;

    try
    {
        _tfStaticTransformBroadcaster->sendTransform(transform);
        ROS_INFO_STREAM("Send new static transform from " << transform.header.frame_id
                                                          << " to " << transform.child_frame_id);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN_STREAM("Can't send new transform" << transform.child_frame_id << ": " << ex.what());
        ret = false;
    }
    return ret;
}

bool TransformationHandler::SendStaticTransform(geometry_msgs::Transform transform, std::string fromFrame,
                                          std::string toFrame)
{
    geometry_msgs::TransformStamped toSend = TransformToStamped(transform, fromFrame, toFrame);
    return SendStaticTransform(toSend);
}

bool TransformationHandler::SendStaticTransform(geometry_msgs::PoseStamped transformPose, std::string toFrame)
{
    geometry_msgs::TransformStamped toSend = PoseToTransformStamped(transformPose.pose, transformPose.header.frame_id,
                                                                    toFrame);
    return SendStaticTransform(toSend);
}

bool TransformationHandler::SendStaticTransform(geometry_msgs::Pose pose, std::string fromFrame, std::string toFrame)
{
    geometry_msgs::TransformStamped toSend = PoseToTransformStamped(pose, fromFrame, toFrame);
    return SendStaticTransform(toSend);
}

geometry_msgs::TransformStamped TransformationHandler::GetTransform(std::string fromFrame, std::string toFrame)
{
    geometry_msgs::TransformStamped transformStamped;

    try
    {
        transformStamped = _tfBuffer.lookupTransform(toFrame, fromFrame, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN_STREAM("Can't get transform from " << fromFrame << " to " << toFrame << " :" << ex.what());
    }

    return transformStamped;
}

geometry_msgs::PoseStamped TransformationHandler::TransformPose(geometry_msgs::Pose pose, std::string fromFrame,
                                                                std::string toFrame)
{
    auto transform = GetTransform(fromFrame, toFrame);
    geometry_msgs::PoseStamped newPose;
    newPose.pose = pose;
    tf2::doTransform(newPose, newPose, transform);
    return newPose;

}

geometry_msgs::PoseStamped TransformationHandler::TransformPose(geometry_msgs::PoseStamped pose, std::string fromFrame,
                                                                std::string toFrame)
{
    auto transform = GetTransform(fromFrame, toFrame);
    geometry_msgs::PoseStamped newPose;
    newPose.pose = pose.pose;
    tf2::doTransform(newPose, newPose, transform);
    return newPose;

}

geometry_msgs::PoseStamped TransformationHandler::TransformPose(geometry_msgs::PoseStamped transformPose,
                                                                std::string toFrame)
{
    auto transform = GetTransform(transformPose.header.frame_id, toFrame);
    geometry_msgs::PoseStamped newPose;
    newPose.pose = transformPose.pose;

    tf2::doTransform(transformPose, newPose, transform);
    return newPose;
}


geometry_msgs::PoseStamped TransformationHandler::TransformPose(geometry_msgs::TransformStamped transform,
                                                                geometry_msgs::PoseStamped pose)
{
    tf2::doTransform(pose, pose, transform);
    return pose;
}

geometry_msgs::Pose TransformationHandler::TransformPose(geometry_msgs::TransformStamped transform,
                                                                geometry_msgs::Pose pose)
{
    geometry_msgs::PoseStamped ret;
    ret.pose = pose;
    tf2::doTransform(ret, ret, transform);
    return pose;
}


bool TransformationHandler::FrameExist(std::string frameName)
{
    bool ret = false;
    try
    {
        ret  = _tfBuffer._frameExists(frameName);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN_STREAM("Can't check frame existance of " << frameName << ex.what());
    }

    return ret;
}

TransformationHandler::~TransformationHandler()
{
    delete _tfListener;
    delete _tfBroadcaster;
    delete _tfStaticTransformBroadcaster;
}