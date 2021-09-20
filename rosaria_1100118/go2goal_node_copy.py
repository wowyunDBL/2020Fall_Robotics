#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from sensor_msgs.msg import PointCloud

import numpy as np
import tf
import csv
import time
import Queue

# simulation parameters
Kp_rho = 0.14
Kp_alpha = 0.3
Kp_beta = 0.2

rho_tolerance = 0.02

theta = float()
poseEgo = Pose()
target = 0.23

line_strip = Marker()
p = Point()


velPub = None
BKPub = None
posSub = None
MPSub = None
bk1Pub = None
bk2Pub = None
bk3Pub = None
bk4Pub = None
bk5Pub = None

flag_poseClbk = False
alpha_flag = False
flag_MP = False
flag_stall = False

y_diff_goal = 0.0 #1.3*np.tan(10.0/180*np.pi)
last5TurnAroundData = Queue.Queue()

def cbMicroPhone(msg):
    global flag_MP
    if flag_MP == False:
        flag_MP = True
        rospy.loginfo("Into cbInitial funciton.")

def turnCar(angle):
    global flag_poseClbk
    global velPub
    if flag_poseClbk == False:
        msgVel = Twist()
        msgVel.linear.x = 0
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = angle
        velPub.publish(msgVel)
        rospy.loginfo("Into cbOdom funciton.")


last_time=time.time()

def cbOdom_getTag(msg):
    global theta, flag_poseClbk, poseEgo, flag_MP, flag_back,last_time
    last_time=time.time()
    flag_back = True
    if flag_MP == True:
        if flag_poseClbk == False:
            flag_poseClbk = True
            rospy.loginfo("Into cbOdom_getTag funciton.")
        # old
        # poseEgo.position.x = msg.pose.pose.position.x
        # poseEgo.position.y = msg.pose.pose.position.y
        # poseEgo.position.z = msg.pose.pose.position.z
        # quaternion = (
        #     msg.pose.pose.orientation.x,
        #     msg.pose.pose.orientation.y,
        #     msg.pose.pose.orientation.z,
        #     msg.pose.pose.orientation.w )
        #new
        poseEgo.position.x = msg.pose.position.x
        poseEgo.position.y = msg.pose.position.y
        poseEgo.position.z = 0
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        theta = euler[2] 
        # print "origin theta: %.2f\n" % (theta)
        if theta < 0:
            theta = np.pi + theta
        else:
            theta = theta - np.pi
        # print "after theta: %.2f\n" % (theta)

def is_time_out():
    global last_time
    if time.time()-last_time>0.3:
        return 1
    else:
        return 0


def stopCar():
    msgVel = Twist()
    msgVel.linear.x = 0
    msgVel.linear.y = 0
    msgVel.linear.z = 0
    msgVel.angular.x = 0
    msgVel.angular.y = 0
    msgVel.angular.z = 0
    velPub.publish(msgVel)
    print("Stop car!")

def turn_around(dist, angle):
    rate = rospy.Rate(5)
    final_angle = -80*np.pi/180/3
    if dist<0:
        dist = -dist
        final_angle = final_angle

    for i in range(15):
        msgVel = Twist()
        msgVel.linear.x = 0
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = angle/3.8
        velPub.publish(msgVel)
        print("turn around")
        rate.sleep()
    for i in range(15):
        msgVel = Twist()
        if dist/4<0.01:
            msgVel.linear.x = 0
        else:
            msgVel.linear.x = dist/4
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = 0
        velPub.publish(msgVel)
        print("go straight")
        rate.sleep()
    for i in range(15):
        msgVel = Twist()
        msgVel.linear.x = 0
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = final_angle
        velPub.publish(msgVel)
        print("turn back")
        rate.sleep()

flag_fine = True

def move_to_pose(x_goal, y_goal, theta_goal):
    global theta, flag_poseClbk, poseEgo, alpha_flag, bk1Sub, flag_stall
    global velPub, flag_fine
    

    if flag_poseClbk == True: 
        rospy.loginfo('Into move_to_pose funciton.')
        
        x = poseEgo.position.x
        y = poseEgo.position.y

        x_diff = -x_goal + x
        y_diff = -y_goal + y
        x_traj, y_traj = [], []

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff) - theta + np.pi) % (2.0 * np.pi) - np.pi # [-pi, pi]
        beta = (theta_goal - theta - alpha + np.pi) % (2.0 * np.pi) - np.pi  # [-pi, pi]

        rate = rospy.Rate(10)
        while (rho > rho_tolerance or np.abs(beta)> 0.09) and (not rospy.is_shutdown()):
            # print "now rho: %.3f m \nnow beta: %.3f" % (rho, np.abs(beta))
            x_traj.append(poseEgo.position.x)
            y_traj.append(poseEgo.position.y)

            x_diff = -x_goal + poseEgo.position.x
            y_diff = -y_goal + poseEgo.position.y
            print(x_diff,y_diff)

            rho = np.hypot(x_diff, y_diff)
            alpha = (np.arctan2(y_diff, x_diff) - theta + np.pi) % (2.0 * np.pi) - np.pi
            beta = (theta_goal - theta + np.pi) % (2.0 * np.pi) - np.pi
            
            # record last 5 (y_diff + y_diff_goal, 1.57-alpha)
            turnAroundData = np.array([y_diff + y_diff_goal, 1.57-alpha])
            if last5TurnAroundData.qsize() < 5:
                last5TurnAroundData.put(turnAroundData)
            else:
                _ = last5TurnAroundData.get()
                last5TurnAroundData.put(turnAroundData)




            if alpha_flag == False:
                beta = 0.0

            # if it is near enough, don't turn head to goal
            if rho < 0.8 or alpha_flag: 
                alpha = 0.0
                alpha_flag = True
            # if goal pose is back or near enough, don't move forward
            if alpha > np.pi / 2 or alpha < -(np.pi / 2.0) or rho < 0.15:
                rho = 0.0

            v = Kp_rho * rho
            w = Kp_alpha * alpha + Kp_beta * beta

            # limit velocity
            if v > 0.2:
                v = 0.2
            if w > 2:
                w = 2
            
            print "now alpha: %.4f  now beta: %.4f \nnow rho: %.4f \nnow v: %.4f now w: %.4f" % (Kp_alpha * alpha, beta, rho, v, w)
            print "poseX: %.3f  poseY: %.3f  theta: %.2f\n" % (poseEgo.position.x, poseEgo.position.y, theta)
            #f = open('/home/anny/pioneer/result5.csv', 'a+')
            #with f:
		        #writer = csv.writer(f)
		        #writer.writerow([Kp_alpha * alpha, Kp_beta * beta, alpha, beta, theta, rho, v, w, x_diff, y_diff])
            msgVel = Twist()
            msgVel.linear.x = v
            msgVel.linear.y = 0
            msgVel.linear.z = 0
            msgVel.angular.x = 0
            msgVel.angular.y = 0
            msgVel.angular.z = w

            if is_time_out():
                stopCar()
            else:
                velPub.publish(msgVel)

            
            if (rho < 0.8 and flag_fine) and np.abs(y_diff)> 0.2:
                n = last5TurnAroundData.qsize()
                temp = []
                for i in range(n):
                    temp.append(last5TurnAroundData.get())
                temp_np = np.array(temp)
                for i in range(n):
                    last5TurnAroundData.put(temp[i])
                meanTurnAroundData = np.mean(temp_np, 0)
                

                turn_around(meanTurnAroundData[0], meanTurnAroundData[1])
                flag_fine = False
            
            rate.sleep()

        print "Final rho: %.2f\n poseX: %.3f  poseY: %.3f  theta: %.2f" % (rho, poseEgo.position.x, poseEgo.position.y, theta)
        print "Finish moving, stop!"
        msgVel = Twist()
        msgVel.linear.x = 0
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = 0
        velPub.publish(msgVel)
        rospy.loginfo("Back to main funciton.")

def cb2(msg):
    global bk2Pub, velPub
    rate = rospy.Rate(5)
    for i in range(10):
        msgVel = Twist()
        msgVel.linear.x = -0.05
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = 0
        velPub.publish(msgVel)
        rate.sleep()
    msgVel = Twist()
    msgVel.linear.x = 0
    msgVel.linear.y = 0
    msgVel.linear.z = 0
    msgVel.angular.x = 0
    msgVel.angular.y = 0
    msgVel.angular.z = 0
    velPub.publish(msgVel)
    bkMsg = Empty()
    bk2Pub.publish(bkMsg)

def cb5(msg):
    global bk5Pub, velPub
    rate = rospy.Rate(5)
    for i in range(10):
        msgVel = Twist()
        msgVel.linear.x = 0.05
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = 0
        velPub.publish(msgVel)
        rate.sleep()
    msgVel = Twist()
    msgVel.linear.x = 0
    msgVel.linear.y = 0
    msgVel.linear.z = 0
    msgVel.angular.x = 0
    msgVel.angular.y = 0
    msgVel.angular.z = 0
    velPub.publish(msgVel)
    bkMsg = Empty()
    bk5Pub.publish(bkMsg)

def cb4(msg):
    global bk4Pub, velPub
    rate = rospy.Rate(5) # 0.2s
    for i in range(30):
        msgVel = Twist()
        msgVel.linear.x = 0.05
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = 0
        velPub.publish(msgVel)
        rate.sleep()
    msgVel = Twist()
    msgVel.linear.x = 0
    msgVel.linear.y = 0
    msgVel.linear.z = 0
    msgVel.angular.x = 0
    msgVel.angular.y = 0
    msgVel.angular.z = 0
    velPub.publish(msgVel)
    bkMsg = Empty()
    bk4Pub.publish(bkMsg)

def cb3(msg):
    global bk3Pub, velPub, flag_stall, alpha_flag, flag_back, flag_poseClbk, flag_fine
    alpha_flag = False
    flag_back = False
    flag_poseClbk = False
    flag_fine = True

    rate = rospy.Rate(5)
    for i in range(40):
        msgVel = Twist()
        msgVel.linear.x = -0.05
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = 0
        velPub.publish(msgVel)
        print("back")
        rate.sleep()

    rate = rospy.Rate(10)
    while(not flag_back):
        msgVel = Twist() #turn car
        msgVel.linear.x = 0
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = -0.2
        velPub.publish(msgVel)
        rate.sleep()
    move_to_pose(0.1, 0, 0)
    
    rate = rospy.Rate(5)
    for i in range(17):
        msgVel = Twist()
        msgVel.linear.x = 0
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = np.pi/3.6
        velPub.publish(msgVel)
        print("origin")
        rate.sleep()
    
    msgVel = Twist()
    msgVel.linear.x = 0
    msgVel.linear.y = 0
    msgVel.linear.z = 0
    msgVel.angular.x = 0
    msgVel.angular.y = 0
    msgVel.angular.z = 0
    velPub.publish(msgVel)
    bkMsg = Empty()
    bk3Pub.publish(bkMsg)

def cbSonar(msg):
    global flag_stall
    tmp = False
    for i in range(8):
        if (msg.points[i].x**2 + msg.points[i].y**2)**(1/2) < 0.1:
            tmp = True
    if tmp:
        flag_stall = True
    else:
        flag_stall = False


def main():
    global velPub, posSub, MPSub, target, flag_poseClbk, flag_MP, bk1Pub, bk3Pub, flag_back, bk2Pub,bk4Pub, bk5Pub
    rospy.init_node('pioneer_controller', anonymous=True)
    velPub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)

    # posSub = rospy.Subscriber('/odom', Odometry, cbOdom)        ## change to another position topic
    posSub = rospy.Subscriber('aruco2xyz', PoseStamped, cbOdom_getTag)        ## change to another position topic
    # MPSub = rospy.Subscriber('/MP_received', Bool, cbMicroPhone)
    bk1Sub = rospy.Subscriber('to_car1', Empty, cbMicroPhone)
    bk1Pub = rospy.Publisher('from_car1', Empty, queue_size=10)

    bk2Sub = rospy.Subscriber('to_car2', Empty, cb2)
    bk2Pub = rospy.Publisher('from_car2', Empty, queue_size=10)

    bk4Sub = rospy.Subscriber('to_car4', Empty, cb4)
    bk4Pub = rospy.Publisher('from_car4', Empty, queue_size=10)

    bk5Sub = rospy.Subscriber('to_car5', Empty, cb5)
    bk5Pub = rospy.Publisher('from_car5', Empty, queue_size=10)

    bk3Sub = rospy.Subscriber('to_car3', Empty, cb3)
    bk3Pub = rospy.Publisher('from_car3', Empty, queue_size=10)
    # sonarSub = rospy.Subscriber('/RosAria/sonar', PointCloud, cbSonar)

    rospy.sleep(2)
    x_goal = target #input('set x goal: ')
    y_goal = 0 #input('set y goal: ')
    theta_goal = 0*np.pi/180 #input('set theta goal: ')
    print("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n" % (x_goal, y_goal, theta_goal))
    while(flag_MP == False and (not rospy.is_shutdown())): 
        print("Waiting for Initialing...")
        rospy.sleep(0.5)
    while(flag_poseClbk == False and (not rospy.is_shutdown())):
        print("Searching for Aruco...")
        turnCar(0.2)
        rospy.sleep(0.5)
    move_to_pose(x_goal, y_goal, theta_goal)
    bkMsg = Empty()
    bk1Pub.publish(bkMsg)

    rospy.spin()

if __name__ == '__main__':
    main()

