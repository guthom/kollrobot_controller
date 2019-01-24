#!/usr/bin/env python
import rospy
import os, sys
projectDir = os.path.join(os.path.dirname(__file__), '../pysrc/')

sys.path.append(projectDir)
sys.path.append(projectDir + "kollrobot_controller/")

from nodes.MesseDemo import MesseDemo


if __name__ == '__main__':
    try:
        messeDemo = MesseDemo()
        messeDemo.node()

    except rospy.ROSInterruptException:
        pass
