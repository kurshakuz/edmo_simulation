#!/usr/bin/python
# General imports
import sys
import rospy

from cpg_control import SERVOMIN, SERVOMAX, constrain, map_range, Oscillator, CPGController


# Execute the application
if __name__ == "__main__":
    rospy.init_node("EDMOEvaluationServer")

