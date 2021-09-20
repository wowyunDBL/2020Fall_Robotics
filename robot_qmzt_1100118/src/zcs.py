#!/usr/bin/env python
import os
import rospy
import serial
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
#ser = serial.Serial('/dev/ttyUSB1',57600,timeout = 0.1)



def send2mega(m1,m2,m3):
    global ser
    s="%04d,%03d,%03d"%(m1,m2,m3)
    print(s.encode())
    #ser.write(s.encode())
    #e_pose = ser.readline().decode().split()
s="-1"
axis1_t=0
def cb_from_m(data):
    global s,axis1_t
    if s=="0":
        axis1_t=200
        send2mega(axis1_t,0,0)
        time.sleep(5)
        to_car1_pub_msg=Empty()
        to_car1_pub.publish(to_car1_pub_msg)
        s="1"

def cb_from_car1(data):
    global s
    if s=="1":
        s="2"
def cb_from_car2(data):
    global s
    if s=="6":
        s="7"
def cb_from_car3(data):
    global s
    if s=="9":
        s="10"
axis1_z=0

def cb_get_z(data):
    global axis1_z
    axis1_z=data.pose.position.z
rospy.init_node('main code', anonymous=True)
from_car1_sub = rospy.Subscriber('from_car1', Empty, cb_from_car1)
from_car2_sub = rospy.Subscriber('from_car2', Empty, cb_from_car2)
from_car3_sub = rospy.Subscriber('from_car3', Empty, cb_from_car3)
from_m_sub = rospy.Subscriber('from_m', Empty, cb_from_m)
get_z = rospy.Subscriber('aruco2xyz', PoseStamped, cb_get_z)

to_car1_pub = rospy.Publisher('to_car1', Empty, queue_size=1)
to_car2_pub = rospy.Publisher('to_car2', Empty, queue_size=1)
to_car3_pub = rospy.Publisher('to_car3', Empty, queue_size=1)

rate = rospy.Rate(10)


while not rospy.is_shutdown():
    if s=="-1":
        print("s-1")
        axis1_t=0
        send2mega(axis1_t,0,0)
        time.sleep(5)

        s="0"
    elif s=="0":
        print("s0")

    elif s=="1":
        print("s1")
    elif s=="2":
        print("s2")
        h=0.1
        e=0.01
        if axis1_z>h+e:
            axis1_t=axis1_t-1
            send2mega(axis1_t,0,0)
            time.sleep(1)
        elif axis1_z<h-e:
            axis1_t=axis1_t+1
            send2mega(axis1_t,0,0)
            time.sleep(1)
        else:
            s="3"
    elif s=="3":
        print("s3")
        s="4"
    elif s=="4":
        print("s4")
        send2mega(axis1_t,0,180)
        time.sleep(5)
        s="5"
    elif s=="5":
        print("s5")
	for i in range(81):
            axis1_t=axis1_t-1
	    send2mega(axis1_t,i,180)
	    time.sleep(0.1)
        s="6"
    elif s=="6":
        print("s6")
    elif s=="7":
        print("s7")
        send2mega(axis1_t,80,0)
        time.sleep(5)
        s="8"
    elif s=="8":
        print("s8")
        send2mega(axis1_t,0,0)
        time.sleep(5)
        s="9"
    elif s=="9":
        print("s9")

    elif s=="10":
        print("s10")
        axis1_t=0
        send2mega(axis1_t,0,0)
        time.sleep(5)
        s="0"


    rate.sleep()
