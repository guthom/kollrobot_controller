//
// Created by thomas on 14.06.18.
//
#pragma once

#include <cstring>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>


class TransformationHandler {

private:
    void Init();

    //tfStuff
    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener* _tfListener;
    tf2_ros::TransformBroadcaster* _tfBroadcaster;
    tf2_ros::StaticTransformBroadcaster* _tfStaticTransformBroadcaster;

public:
    TransformationHandler();
    ~TransformationHandler();

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

    geometry_msgs::TransformStamped GetTransform(std::string fromFrame, std::string toFrame);

    bool FrameExist(std::string frameName);



};
