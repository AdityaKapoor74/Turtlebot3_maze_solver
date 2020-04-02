#!/usr/bin/env python
import sys
import os
sys.path.append(os.getcwd())
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'helper')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'policy/')))

import rospy

from scan_twist_center_control_ import ScanTwistCenterControlPolicyNode
from pd_controller import PDController


rospy.init_node("ROS_maze_bot")

kwargs = {"distance_to_wall_desired":0.2, "max_speed":0.2}

scan_monitor = ScanTwistCenterControlPolicyNode(scan_topic="/scan",pub_topic="cmd_vel",policy="LHR",helper_controller=PDController,**kwargs)

if __name__=="__main__":
	scan_monitor.start()