import sys, os
import rospy

#find better way to do this.
projectDir = os.path.join(os.path.dirname(__file__), '../')

sys.path.append(projectDir)

import nodeconfig as config

from kollrobot_controller.msg._PlaceBoxActionGoal import *

#ros stuff
import tf2_ros

class KollrobotController:

    _publisher = dict()
    _tfBuffer = None

    def Init(self):
        # init node disable signal for internal shutdown
        rospy.init_node(config.nodeName, anonymous=True, disable_signals=True)

        # tfStuff
        self._tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tfBuffer)


    def node(self):

        self.Init()

        rate = rospy.Rate(config.rate)

        print("Started Node: " + config.nodeName)
        while not rospy.is_shutdown():


            rate.sleep()

