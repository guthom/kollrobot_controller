import sys, os
import rospy
import functools
import nodeconfig_messe as config
import actionlib

from kollrobot_controller.msg import *
from kollrobot_controller.srv import *

class MesseDemo:

    actGoPosition = None
    actPickBoxAction = None
    actPlaceBoxAction = None
    srvCheckPosition = None
    srvCheckTrajectoryExecution = None

    def Init(self):
        # init node disable signal for internal shutdown
        rospy.init_node(config.nodeName, anonymous=True, disable_signals=True)

        self.actGoPosition = actionlib.SimpleActionClient(config.goPositionTopic, GoPositionAction)
        self.actPickBoxAction = actionlib.SimpleActionClient(config.pickBoxTopic, PickBoxAction)
        self.actPlaceBoxAction = actionlib.SimpleActionClient(config.placeBoxTopic, PlaceBoxAction)


        rospy.loginfo("Waiting for services!")
        rospy.wait_for_service(config.checkPositionService)
        self.srvCheckPosition = rospy.ServiceProxy(config.checkPositionService, CheckPosition)
        rospy.wait_for_service(config.checkTrajectoryService)
        self.srvCheckTrajectoryExecution = rospy.ServiceProxy(config.checkTrajectoryService, IsTrajectoryExecuting)
        rospy.loginfo("Needed services are available!")


    def PickBox(self, id):
        rospy.loginfo("Execute PickBoxAction to " + "boxApril_" + str(id))
        msg = PickBoxActionGoal()

        msg.goal.box_frameID = "boxApril_" + str(id)
        rospy.sleep(1)
        self.actPickBoxAction.send_goal(msg.goal)
        self.actPickBoxAction.wait_for_result()


    def GoPosition(self, position):
        rospy.loginfo("Execute GoPosition to " + position)
        msg = GoPositionActionGoal()
        msg.goal.position = position
        rospy.sleep(1)
        self.actGoPosition.send_goal(msg.goal)
        self.actGoPosition.wait_for_result()

    def PlaceBox(self, id):
        rospy.loginfo("Execute PlaceBoxAction to " + "boxApril_" + str(id))
        msg = PlaceBoxActionGoal()
        msg.goal.place_frameID = "boxApril_" + str(id)
        rospy.sleep(1)
        self.actPlaceBoxAction.send_goal(msg.goal)
        self.actPlaceBoxAction.wait_for_result()

    def CheckTrajectory(self):

        rqst = IsTrajectoryExecutingRequest()

        while self.srvCheckTrajectoryExecution.call(rqst).result:
            rospy.loginfo("Trajectory is still in progress!")
            rospy.sleep(2)



    def CheckPosition(self, position):

        rospy.loginfo("Check Position at " + position)
        rqst = CheckPositionRequest()
        rqst.positionName = position

        count = 0
        while not self.srvCheckPosition.call(rqst).result:
            rospy.loginfo("Position " + str(position) + " not reached yet!")
            count += 1
            rospy.sleep(2)
            if count > config.posReachedRetry:
                rospy.loginfo("Checked " + str(config.posReachedRetry) + " times, will set position "
                              + str(position) + " again!")
                self.GoPosition(position)

        rospy.loginfo("Robot reached position " + position)


    def CreateControllChain(self):
        chain = list()

        chain.append(functools.partial(self.GoPosition, "drive"))
        chain.append(functools.partial(self.CheckPosition, "drive"))

        chain.append(functools.partial(self.GoPosition, "home"))
        chain.append(functools.partial(self.CheckPosition, "home"))

        chain.append(functools.partial(self.GoPosition, "save_front"))
        chain.append(functools.partial(self.CheckPosition, "save_front"))

        chain.append(functools.partial(self.PickBox, config.boxId))
        chain.append(functools.partial(self.CheckTrajectory))

        chain.append(functools.partial(self.PlaceBox, config.placeID))
        chain.append(functools.partial(self.CheckTrajectory))

        chain.append(functools.partial(self.GoPosition, "save_front"))
        chain.append(functools.partial(self.CheckPosition, "save_front"))

        chain.append(functools.partial(self.PickBox, config.boxId))
        chain.append(functools.partial(self.CheckTrajectory))

        chain.append(functools.partial(self.GoPosition, "save_front"))
        chain.append(functools.partial(self.CheckPosition, "save_front"))

        chain.append(functools.partial(self.PlaceBox, config.tableID))
        chain.append(functools.partial(self.CheckTrajectory))

        chain.append(functools.partial(self.GoPosition, "save_front"))
        chain.append(functools.partial(self.CheckPosition, "save_front"))

        chain.append(functools.partial(self.GoPosition, "home"))
        chain.append(functools.partial(self.CheckPosition, "home"))

        chain.append(functools.partial(self.GoPosition, "drive"))
        chain.append(functools.partial(self.CheckPosition, "drive"))

        return chain

    def RunChain(self, chain):

        for element in chain:
            element()

    def node(self):
        print("Starting Node: " + config.nodeName)
        self.Init()

        chain = self.CreateControllChain()

        print("Started Node: " + config.nodeName)
        counter = 0
        while not rospy.is_shutdown():
            counter += 1
            self.RunChain(chain)
            rospy.logwarn("Finished Chain Nr: " + str(counter))
