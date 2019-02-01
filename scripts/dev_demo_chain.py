#!/usr/bin/env python
import rospy
import os, sys
projectDir = os.path.join(os.path.dirname(__file__), '../pysrc/')

sys.path.append(projectDir)
sys.path.append(projectDir + "kollrobot_controller/")

from nodes.DevDemo import DevDemo


if __name__ == '__main__':
    try:
        DevDemo = DevDemo()
        DevDemo.node()

    except rospy.ROSInterruptException:
        pass
