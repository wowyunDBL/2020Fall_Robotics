#!/usr/bin/env python
import os
import rospy
import serial
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
ser = serial.Serial('/dev/ttyUSB1',57600,timeout = 0.1)



def send2mega(m1,m2,m3):
    global ser
    s="%04d,%03d,%03d"%(m1,m2,m3)
    print(s.encode())
    ser.write(s.encode())
    time.sleep(1)
    print(s.encode())
    ser.write(s.encode())
    time.sleep(3)
    #e_pose = ser.readline().decode().split()

s="init"
axis1_t=0
SLEEP_TIME=2
def cb_from_m(data):
    global s,axis1_t
    if s=="a":
        time.sleep(2)
        s="b"

def cb_from_car1(data):
    global s
    if s=="f":
        s="g"
def cb_from_car2(data):
    global s
    if s=="d":
        s="e"
def cb_from_car3(data):
    global s
    if s=="h":
        s="i"
axis1_z=0

def cb_get_z(data):
    global axis1_z,last_time
    axis1_z=data.pose.position.z
    last_time=time.time()

def aruco_time_out():
    global last_time
    if time.time()-last_time>2:
        return 1
    else:
        return 0


rospy.init_node('main_code_W', anonymous=True)
from_car1_sub = rospy.Subscriber('from_car1', Empty, cb_from_car1)
from_car2_sub = rospy.Subscriber('from_car2', Empty, cb_from_car2)
from_car3_sub = rospy.Subscriber('from_car3', Empty, cb_from_car3)
from_m_sub = rospy.Subscriber('from_m', Empty, cb_from_m)
get_z = rospy.Subscriber('aruco2xyz', PoseStamped, cb_get_z)

to_car1_pub = rospy.Publisher('to_car1', Empty, queue_size=1)
to_car2_pub = rospy.Publisher('to_car2', Empty, queue_size=1)
to_car3_pub = rospy.Publisher('to_car3', Empty, queue_size=1)

rate = rospy.Rate(10)

ru865j83=75
while not rospy.is_shutdown():
    if s=="init":
        print("s_init")
        print("sleep")
        time.sleep(10)
        print("HI")
        axis1_t=-200
        send2mega(axis1_t,0,0)
        time.sleep(SLEEP_TIME)
        s="a"

    elif s=="a":
        print("sa")

    elif s=="b":
        print("sb")
        send2mega(axis1_t,0,ru865j83)
        time.sleep(SLEEP_TIME)
        s="c"

    elif s=="c":
        print("sc")
        axis1_t=-150
        send2mega(axis1_t,0,ru865j83)
        time.sleep(SLEEP_TIME)
        to_car2_pub_msg=Empty()
        to_car2_pub.publish(to_car2_pub_msg)
        s="d"

    elif s=="d":
        print("sd")
        s="e"
    elif s=="e":
        print("se")
        axis1_t=200
        send2mega(axis1_t,0,0)
        time.sleep(SLEEP_TIME)
        to_car1_pub_msg=Empty()
        to_car1_pub.publish(to_car1_pub_msg)
        s="f"

    elif s=="f":
        print("sf")

    elif s=="g":
        print("sg")
        axis1_t=150
        send2mega(axis1_t,0,ru865j83)
        time.sleep(SLEEP_TIME)
        while not aruco_time_out():
            pass        
        s="h"
        
    elif s=="h":
        print("sh")
        send2mega(axis1_t,0,0)
        time.sleep(SLEEP_TIME)
        to_car3_pub_msg=Empty()
        to_car3_pub.publish(to_car3_pub_msg)

    elif s=="i":
        print("si")
        time.sleep(SLEEP_TIME)
        axis1_t=0
        send2mega(axis1_t,0,0)
        time.sleep(SLEEP_TIME)
        s="j"


    rate.sleep()
