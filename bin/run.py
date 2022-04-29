#!/usr/bin/env python
import sys
import rospy
from puzzlebot_control.roswrap import RosWrap 

if __name__ == "__main__":
    try:
        rc = RosWrap(int(sys.argv[1]), use_ui=False)
        rc.start()
    except rospy.ROSInterruptException:
        pass
