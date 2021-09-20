#!/usr/bin/env python
import os
import rospy

import time
from std_msgs.msg import Empty
from std_msgs.msg import UInt8

from geometry_msgs.msg import PoseStamped
s="1"

def cb_from_m(data):
    global s
    s="1"
def cb_to_car(data):
    global s
    if data.data==1:
        s="1"
def cb_to_sw(data):
    global s
    if data.data==123:
        s="2"

def cb_get_1(data):
    global s
    if s=="1":
        marker_pub.publish(data)

def cb_get_2(data):
    global s
    if s=="2":
        marker_pub.publish(data)


rospy.init_node('aruco2xyz_switch', anonymous=True)
#from_car2_sub = rospy.Subscriber('/finish_from_car', UInt8, cb_from_car)
to_car3_sub = rospy.Subscriber('/control_mode', UInt8, cb_to_car)
to_car2_sub = rospy.Subscriber('/control_mode_v2', UInt8, cb_to_sw)
get_1 = rospy.Subscriber('aruco2xyz1', PoseStamped, cb_get_1)
get_2 = rospy.Subscriber('aruco2xyz2', PoseStamped, cb_get_2)
marker_pub=rospy.Publisher("aruco2xyz",PoseStamped,queue_size=1)

while not rospy.is_shutdown():
    rospy.spin()
