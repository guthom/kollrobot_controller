//
// Created by thomas on 14.06.18.
//
#pragma once

#include <cstring>
#include <boost/thread/thread.hpp>
#include <boost/signals2.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
class TransformationHandler {

private:
    void Init();
    //ros stuff
    int _iRefreshRate;
    std::string _nodeName = "TransformationHandler";
    ros::NodeHandle* _node;
    boost::thread* _nodeThread;

    void RunThread();
    void Run();

    //tfStuff
    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener* _tfListener;
    tf2_ros::TransformBroadcaster* _tfBroadcaster;
    tf2_ros::StaticTransformBroadcaster* _tfStaticTransformBroadcaster;

public:
    TransformationHandler(ros::NodeHandle* parentNode, int refreshRate = 30);
    ~TransformationHandler();

    boost::signals2::signal<void()> sigNodeCircle;

    static geometry_msgs::Transform PoseToTransform(geometry_msgs::Pose pose);

    static geometry_msgs::TransformStamped TransformToStamped(geometry_msgs::Transform transform,
                                                              std::string fromFrame,
                                                              std::string toFrame);

    static geometry_msgs::TransformStamped PoseToTransformStamped(geometry_msgs::Pose pose, std::string fromFrame,
                                                                  std::string toFrame);


    bool SendTransform(geometry_msgs::TransformStamped transform);
    bool SendTransform(geometry_msgs::Transform transform, std::string fromFrame, std::string toFrame);
    bool SendTransform(geometry_msgs::PoseStamped transformPose, std::string toFrame);
    bool SendTransform(geometry_msgs::Pose pose, std::string fromFrame, std::string toFrame);

    bool SendStaticTransform(geometry_msgs::TransformStamped transform);
    bool SendStaticTransform(geometry_msgs::Transform transform, std::string fromFrame, std::string toFrame);
    bool SendStaticTransform(geometry_msgs::PoseStamped transformPose, std::string toFrame);
    bool SendStaticTransform(geometry_msgs::Pose pose, std::string fromFrame, std::string toFrame);

    geometry_msgs::PoseStamped TransformPose(geometry_msgs::Pose pose, std::string fromFrame, std::string toFrame);
    geometry_msgs::PoseStamped TransformPose(geometry_msgs::PoseStamped transformPose, std::string toFrame);
    geometry_msgs::PoseStamped TransformPose(geometry_msgs::TransformStamped transform, geometry_msgs::PoseStamped);
    geometry_msgs::PoseStamped TransformPose(geometry_msgs::PoseStamped pose, std::string fromFrame,
                                             std::string toFrame);

    geometry_msgs::Pose TransformPose(geometry_msgs::TransformStamped transform, geometry_msgs::Pose);


    geometry_msgs::TransformStamped GetTransform(std::string fromFrame, std::string toFrame);

    bool FrameExist(std::string frameName);



};
