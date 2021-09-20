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
    #e_pose = ser.readline().decode().split()
s="-1"
axis1_t=0

SLEEP_TIME=3
def cb_from_m(data):
    global s,axis1_t
    if s=="0":
        axis1_t=200
        send2mega(axis1_t,90,0)
        time.sleep(SLEEP_TIME)
        to_car1_pub_msg=Empty()
        to_car1_pub.publish(to_car1_pub_msg)
        s="1"

def cb_from_car1(data):
    global s
    if s=="1":
        s="2"
def cb_from_car2(data):
    global s
    if s=="6" or s=="5":
        print("to_Car4")
        to_car4_pub_msg=Empty()
        to_car4_pub.publish(to_car4_pub_msg)
def cb_from_car3(data):
    global s
    if s=="9":
        s="10"
def cb_from_car4(data):
    global s
    if s=="6" or s=="5":
        s="7"
def cb_from_car5(data):
    global s
    if s=="3":
        s="4"
axis1_z=0

def cb_get_z(data):
    global axis1_z
    axis1_z=data.pose.position.z

rospy.init_node('main code', anonymous=True)
from_car1_sub = rospy.Subscriber('from_car1', Empty, cb_from_car1)
from_car2_sub = rospy.Subscriber('from_car2', Empty, cb_from_car2)
from_car3_sub = rospy.Subscriber('from_car3', Empty, cb_from_car3)
from_car4_sub = rospy.Subscriber('from_car4', Empty, cb_from_car4)
from_car5_sub = rospy.Subscriber('from_car5', Empty, cb_from_car5)
from_m_sub = rospy.Subscriber('from_m', Empty, cb_from_m)
get_z = rospy.Subscriber('aruco2xyz', PoseStamped, cb_get_z)

to_car1_pub = rospy.Publisher('to_car1', Empty, queue_size=1)
to_car2_pub = rospy.Publisher('to_car2', Empty, queue_size=1)
to_car3_pub = rospy.Publisher('to_car3', Empty, queue_size=1)
to_car4_pub = rospy.Publisher('to_car4', Empty, queue_size=1)
to_car5_pub = rospy.Publisher('to_car5', Empty, queue_size=1)
rate = rospy.Rate(10)


while not rospy.is_shutdown():
    if s=="-1":
        print("s-1")
        time.sleep(10)
        axis1_t=200
        send2mega(axis1_t,90,0)
        time.sleep(SLEEP_TIME)
        s="0"

    elif s=="0":
        print("s0")

    elif s=="1":
        print("s1")

    elif s=="2":
        print("s2")
        h=-0.2
        e=0.03
        print(axis1_z)
        if axis1_z>(h+e):
            axis1_t=axis1_t-10
            send2mega(axis1_t,90,0)
            time.sleep(SLEEP_TIME)
        elif axis1_z<(h-e):
            axis1_t=axis1_t+10
            send2mega(axis1_t,90,0)
            time.sleep(SLEEP_TIME)
        else:
            s="3"
            to_car5_pub_msg=Empty()
            to_car5_pub.publish(to_car5_pub_msg)

    elif s=="3":
        print("s3")

    elif s=="4":
        print("s4")
        send2mega(axis1_t,90,180)
        time.sleep(SLEEP_TIME)
        s="5"

    elif s=="5":
        print("s5")
        axis1_t=axis1_t-90
        send2mega(axis1_t,45,180)
        time.sleep(SLEEP_TIME)
        to_car2_pub_msg=Empty()
        to_car2_pub.publish(to_car2_pub_msg)

        axis1_t=axis1_t-90
        send2mega(axis1_t,0,180)
        time.sleep(SLEEP_TIME)
        s="6"
    elif s=="6":
        print("s6")

    elif s=="7":
        print("s7")
        send2mega(axis1_t,0,0)
        time.sleep(SLEEP_TIME)
        s="8"

    elif s=="8":
        print("s8")
        axis1_t=200
        send2mega(axis1_t,90,0)
        time.sleep(SLEEP_TIME)
        to_car3_pub_msg=Empty()
        to_car3_pub.publish(to_car3_pub_msg)
        s="9"
        
    elif s=="9":
        print("s9")

    elif s=="10":
        print("s10")
        axis1_t=0
        send2mega(axis1_t,90,0)
        time.sleep(SLEEP_TIME)
        s="11"
    elif s == "11":
        break

    rate.sleep()


print("axis1 to zero...")
send2mega(0, 0, 0)
time.sleep(1)
send2mega(0, 0, 0)
time.sleep(5)


    
    

