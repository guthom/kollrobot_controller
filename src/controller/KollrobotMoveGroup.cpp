#include "KollrobotMoveGroup.h"

KollrobotMoveGroup::KollrobotMoveGroup(ros::NodeHandle* parentNode, customparameter::ParameterHandler* parameterHandler, std::string groupName)
    : _parameterHandler(parameterHandler), _groupName(groupName)
{
    Init(parentNode);
}

void KollrobotMoveGroup::Init(ros::NodeHandle* parentNode)
{
    _nodeName = "MG_" + _groupName;
    _node = new ros::NodeHandle(*parentNode, _nodeName);

    _transformationHandler = new TransformationHandler(_node);

    InitParameter();

    //init MoveIT Stuff
    _moveGroup = new moveit::planning_interface::MoveGroupInterface(_groupName);
    _moveGroup->setStartStateToCurrentState();
    //TODO: Velocity Hack! Add Parameter for this
    _moveGroup->setMaxVelocityScalingFactor(double(_paramMaxVelocityScale.GetValue()));
    _moveGroup->setMaxAccelerationScalingFactor(double(_paramMaxAccelerationScale.GetValue()));
    _moveGroup->setPlanningTime(_paramPlanningTime.GetValue());

    _planningSceneInterface = new moveit::planning_interface::PlanningSceneInterface();
    _planningScene = new planning_scene::PlanningScene(_moveGroup->getRobotModel());

    //init publisher
    _pubTargetPose = _node->advertise<visualization_msgs::Marker>(_node->getNamespace() + "/TargetPose", 100);
    _pubWaypoints = _node->advertise<visualization_msgs::MarkerArray>(_node->getNamespace() + "/Waypoints", 100);

    //run node
    _nodeThread = new boost::thread(boost::bind(&KollrobotMoveGroup::Run,this));


    //GoHome();
    SetConstraints();

    InitMarker();
}

void KollrobotMoveGroup::SetConstraints()
{


    if(_param_SetConstraints.GetValue()) {
        _constraints.name = "BoxUP";
        //set constraint to keep box
        /*
        moveit_msgs::JointConstraint jc;
        jc.joint_name = "wrist_2_joint";
        jc.position = -1.58;
        jc.tolerance_above = 0.5;
        jc.tolerance_below = 0.5;
        jc.weight = 1.0;
        _constraints.joint_constraints.push_back(jc);
        */

        std::vector <std::string> linkNames = _moveGroup->getLinkNames();
        geometry_msgs::PoseStamped currentPose = _moveGroup->getCurrentPose(_moveGroup->getEndEffectorLink());
        currentPose = _transformationHandler->TransformPose(currentPose, "world", "base_link");
        //_transformationHandler->SendStaticTransform(currentPose, "TestPOse");
        moveit_msgs::OrientationConstraint ocm;

        ocm.link_name = "ee_link";
        ocm.header.frame_id = "base_link";
        ocm.header.stamp = ros::Time::now();
        ocm.orientation = currentPose.pose.orientation;

        ocm.orientation.x = 0.0;
        ocm.orientation.y = 0.0;
        ocm.orientation.z = 0.0;
        ocm.orientation.w = 1.0;

        ocm.absolute_x_axis_tolerance = 2 * M_PI;
        ocm.absolute_y_axis_tolerance = 2 * M_PI;
        ocm.absolute_z_axis_tolerance = 2 * M_PI; //ignore this axis

        ocm.weight = 0.9;
        _constraints.orientation_constraints.push_back(ocm);

        _moveGroup->setPathConstraints(_constraints);
    }

    //creat hacked scene for save planning with kollrobot
    moveit_msgs::CollisionObject co;
    co.id = "kollrobotApprox";
    co.operation = co.ADD;
    shape_msgs::SolidPrimitive primitive;

    //add first primitive box
    primitive.type = primitive.BOX;
    geometry_msgs::Pose box_pose;

    float securityRange = _paramSecurityRange.GetValue();

    float robOffset[3] = {-0.235f, 0.0f, 0.0f};
    float bigBox[3] = {0.65f + securityRange, 0.56f + securityRange, 0.90f};
    float smallBox[3] = {0.25f + securityRange/4, 0.56f + securityRange, 0.095f};
    float handle[3] = {0.03f + securityRange/4, 0.175f + securityRange, 0.045f};


    primitive.dimensions.resize(3);
    primitive.dimensions[0] = bigBox[0];
    primitive.dimensions[1] = bigBox[1];
    primitive.dimensions[2] = bigBox[2];
    box_pose.orientation.w = 1.0;
    box_pose.position.x = robOffset[0];
    box_pose.position.y = robOffset[1];
    box_pose.position.z = robOffset[2] - bigBox[2]/2;

    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(box_pose);

    //add second primitive box
    primitive.dimensions[0] = smallBox[0];
    primitive.dimensions[1] = smallBox[1];
    primitive.dimensions[2] = smallBox[2];
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -(bigBox[0]/2 - smallBox[0]/2 - securityRange/2 - robOffset[0]) ;
    box_pose.position.y = robOffset[1];
    box_pose.position.z = robOffset[2] + smallBox[2]/2;

    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(box_pose);

    //add handle primitive box
    primitive.dimensions[0] = handle[0];
    primitive.dimensions[1] = handle[1];
    primitive.dimensions[2] = handle[2];
    box_pose.orientation.w = 1.0;
    box_pose.position.x += 0.025;
    box_pose.position.z += smallBox[2]/2 + handle[2]/2;

    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(box_pose);


    ROS_INFO("Added faked groundplane for ROPOSE!");
    _planningSceneInterface->applyCollisionObject(co);

}

void KollrobotMoveGroup::InitParameter()
{
    std::string subNamespace = _nodeName + "/";
    _param_RefreshRate = _parameterHandler->AddParameter("RefreshRate", subNamespace, "", int(20));
    _param_SetConstraints = _parameterHandler->AddParameter("SetConstraints", subNamespace, "", true);
    _paramMaxAccelerationScale = _parameterHandler->AddParameter("MaxAccelerationScale", subNamespace, "", 0.1f);
    _paramMaxVelocityScale = _parameterHandler->AddParameter("MaxVelocityScale", subNamespace, "", 0.1f);
    _paramPlanningTime = _parameterHandler->AddParameter("PlanningTime", subNamespace, "", 3.0f);
    _paramSecurityRange = _parameterHandler->AddParameter("SecurityRange", subNamespace, "", 0.02f);
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
}

void KollrobotMoveGroup::PlanToPose(geometry_msgs::PoseStamped targetPose)
{
    _moveGroup->setPoseTarget(targetPose);
    _markerTargetPose.pose = targetPose.pose;
    _markerTargetPose.header.frame_id = targetPose.header.frame_id;

    RunPlanning();
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
}


void KollrobotMoveGroup::PlanToPositionExecute(geometry_msgs::PointStamped targetPose)
{
    _moveGroup->clearPoseTargets();
    _moveGroup->setPositionTarget(targetPose.point.x,targetPose.point.y,targetPose.point.z);
    _markerTargetPose.pose.position = targetPose.point;
    RunPlanningExecute();
}

void KollrobotMoveGroup::PlanToPositionExecute(geometry_msgs::Point targetPose)
{
    _moveGroup->clearPoseTargets();
    _moveGroup->setPositionTarget(targetPose.x,targetPose.y,targetPose.z);
    _markerTargetPose.pose.position = targetPose;
    RunPlanningExecute();
}

void KollrobotMoveGroup::ExecuteTrajectory(moveit_msgs::RobotTrajectory trajectory)
{
    _currentPlan.trajectory_ = trajectory;
    Execute();
}

void KollrobotMoveGroup::ExecutePoseSeries(std::vector<geometry_msgs::PoseStamped> poses)
{
    visualization_msgs::MarkerArray waypointMarker = CreateWaypointMarker(poses);
    _pubWaypoints.publish(waypointMarker);

    for(int i = 0; i < poses.size(); i++)
    {
        PlanToPose(poses[i]);
        Execute();
    }

}


visualization_msgs::MarkerArray KollrobotMoveGroup::CreateWaypointMarker(std::vector<geometry_msgs::Pose> waypoints,
                                                                         std::string frameID="base_link")
{
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker baseMarker;
    baseMarker.id = 0;
    baseMarker.header.frame_id = frameID;
    baseMarker.type = visualization_msgs::Marker::SPHERE;
    baseMarker.scale.x = 0.03;
    baseMarker.scale.y = 0.03;
    baseMarker.scale.z = 0.03;
    baseMarker.color.a = 1.0;
    baseMarker.color.r = 0.0;
    baseMarker.color.g = 0.0;
    baseMarker.color.b = 1.0;

    for(int i = 0; i < waypoints.size(); i++)
    {
        baseMarker.id += 1;
        baseMarker.pose = waypoints[i];
        markerArray.markers.push_back(baseMarker);
    }

    return markerArray;
}

visualization_msgs::MarkerArray KollrobotMoveGroup::CreateWaypointMarker(
        std::vector<geometry_msgs::PoseStamped> waypoints)
{
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker baseMarker;
    baseMarker.id = 0;
    baseMarker.type = visualization_msgs::Marker::SPHERE;
    baseMarker.scale.x = 0.03;
    baseMarker.scale.y = 0.03;
    baseMarker.scale.z = 0.03;
    baseMarker.color.a = 1.0;
    baseMarker.color.r = 0.0;
    baseMarker.color.g = 0.0;
    baseMarker.color.b = 1.0;

    for(int i = 0; i < waypoints.size(); i++)
    {
        baseMarker.id += 1;
        baseMarker.pose = waypoints[i].pose;
        baseMarker.header.frame_id = waypoints[i].header.frame_id;
        markerArray.markers.push_back(baseMarker);
    }

    return markerArray;
}

moveit_msgs::RobotTrajectory KollrobotMoveGroup::ComputeCartesianpath(std::vector<geometry_msgs::Pose> waypoints)
{
    visualization_msgs::MarkerArray waypointMarker = CreateWaypointMarker(waypoints);
    _pubWaypoints.publish(waypointMarker);

    moveit_msgs::RobotTrajectory trajectory;
    trajectory.joint_trajectory.header.frame_id = "simulatedQR";

    _moveGroup->computeCartesianPath(waypoints, 0.005, 0.0, trajectory, _moveGroup->getPathConstraints());

    return trajectory;
}

void KollrobotMoveGroup::PlanToPoseExecute(geometry_msgs::PoseStamped targetPose)
{
    _moveGroup->setPoseTarget(targetPose);
    _markerTargetPose.header.frame_id = targetPose.header.frame_id;
    _markerTargetPose.pose = targetPose.pose;

    RunPlanningExecute();
}

bool KollrobotMoveGroup::GoHome()
{
    return GoPosition("home");
}


bool KollrobotMoveGroup::GoPosition(std::string positionName)
{
    bool poseValid = _moveGroup->setNamedTarget(positionName);
    if(!poseValid)
    {
        ROS_WARN_STREAM("The movegroup does not know the pose: " + positionName + " Add it to the move group config");
        return false;
    }

    RunPlanningExecute();
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
}

void KollrobotMoveGroup::MoveToValidRandomRun()
{

    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = _groupName;
    collision_detection::CollisionResult collision_result;
    robot_state::RobotState currentState = _planningScene->getCurrentState();
    do
    {
        currentState = _planningScene->getCurrentState();
        do
        {
            collision_result.clear();
            currentState.setToRandomPositions();
            _planningScene->checkSelfCollision(collision_request, collision_result, currentState);
            ROS_INFO_STREAM("Collision Check for self Collision pose = " << collision_result.collision);

        }while (collision_result.collision);

        collision_result.clear();
        _planningScene->checkCollision(collision_request, collision_result, currentState);
        ROS_INFO_STREAM("Collision Check for random pose = " << collision_result.collision);
    }
    while (collision_result.collision);

    _moveGroup->setJointValueTarget(currentState);

    //_moveGroup->setRandomTarget();

    IsExecuting = true;
    _moveGroup->move();
    IsExecuting = false;
}

bool KollrobotMoveGroup::IsBusy()
{
    return IsExecuting || IsPlanning;
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
