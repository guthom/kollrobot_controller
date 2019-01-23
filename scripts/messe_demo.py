import rospy
import os, sys
projectDir = os.path.join(os.path.dirname(__file__), '../pysrc/')

sys.path.append(projectDir)
sys.path.append(projectDir + "kollrobot_controller/")


from pysrc.kollrobot_controller.nodes.KollrobotController import KollrobotController


if __name__ == '__main__':
    try:
        kollrobotController = KollrobotController()

        kollrobotController.node()


    except rospy.ROSInterruptException:
        pass
