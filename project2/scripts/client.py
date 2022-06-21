#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from project2.srv import SaveTrajectory


def trajectory_saver_client(x, y):
    rospy.wait_for_service('savetrajectory')
    try:
        trajectory_saver = rospy.ServiceProxy('savetrajectory', SaveTrajectory)
        trajectory_saver()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    if len(sys.argv) != 1:
        print("Error in saving the map")
        sys.exit(1)
