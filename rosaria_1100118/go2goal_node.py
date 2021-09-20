#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np
import tf
import csv
import time

# simulation parameters
Kp_rho = 0.14
Kp_alpha = 0.4
Kp_beta = 0.1

theta = float()
poseEgo = Pose()

line_strip = Marker()
p = Point()
#time_past = float()
#time_now = float()

flag = False
alpha_flag = False

def cbOdom(msg):
    global theta, flag, poseEgo
    rospy.loginfo("into cbOdom funciton.")
    if flag == False:
        flag = True
        #rospy.loginfo("into cbOdom funciton.")
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
    print "now x: %.4f now :y %.4f now theta: %.4f\n\n" % (poseEgo.position.x, poseEgo.position.y, theta)

rospy.init_node('poineer_controller', anonymous=True)
velPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
posSub = rospy.Subscriber('aruco2xyz', PoseStamped, cbOdom)

#update_pose
    
def move_to_pose(x_goal, y_goal, theta_goal):
    global theta, flag, poseEgo, alpha_flag
    """
    rho is the distance between the robot and the goal position
    alpha is the angle to the goal relative to the heading of the robot
    beta is the angle between the robot's position and the goal position plus the goal angle
    Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards the goal
    Kp_beta*beta rotates the line so that it is parallel to the goal angle
    """
    if flag == True: 
        rospy.loginfo('into move_to_pose funciton.')
        print "move to pose"
        x = poseEgo.position.x
        y = poseEgo.position.y

        x_diff = -x_goal + x
        y_diff = y_goal - y

        x_traj, y_traj = [], []

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff) - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi

        rate = rospy.Rate(100)
        while np.abs(beta) > 0.08 or rho > 0.1:
            print "now rho: %.3f m now while: %.3f" % (rho, np.abs(beta))
            x_traj.append(poseEgo.position.x)
            y_traj.append(poseEgo.position.y)

            x_diff = x_goal - poseEgo.position.x
            y_diff = y_goal - poseEgo.position.y

            # Restrict alpha and beta (angle differences) to the range
            # [-pi, pi] to prevent unstable behavior e.g. difference going
            # from 0 rad to 2*pi rad with slight turn

            rho = np.hypot(x_diff, y_diff)
            alpha = (np.arctan2(y_diff, x_diff) - theta + np.pi) % (2 * np.pi) - np.pi
            beta = (np.pi-(theta_goal - theta) + np.pi) % (2 * np.pi) - np.pi

            if rho < 0.1 or alpha_flag: 
                alpha = 0
                alpha_flag = True

            if alpha > np.pi / 2 or alpha < -(np.pi / 2) or rho < 0.1:
                rho = 0

            v = Kp_rho * rho
            w = Kp_alpha * alpha  - beta * Kp_beta

            if v > 0.1:
                v = 0.1
            if w > 1.5:
                w = 1.5
            print "now alpha: %.4f now beta: %.4f now rho: %.4f \nnow v: %.4f now w: %.4f \n\n" % (Kp_alpha * alpha, beta, rho, v, w)
            msgVel = Twist()
            
            msgVel.linear.x = v
            msgVel.linear.y = 0
            msgVel.linear.z = 0
            msgVel.angular.x = 0
            msgVel.angular.y = 0
            msgVel.angular.z = w
            velPub.publish(msgVel)
            rate.sleep()
        print "final rho: %.2f  theta: %.2f" % (rho,theta)
        msgVel = Twist()
        msgVel.linear.x = 0
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = 0
        velPub.publish(msgVel)
        rospy.loginfo("back to main funciton.")

def main():
    
    x_goal = 0.5
    y_goal = 0
    theta_goal = 0
    rospy.sleep(3)
    #print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n" % (x_start, y_start, theta_start))
    print("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n" %
            (x_goal, y_goal, theta_goal))
    move_to_pose(x_goal, y_goal, theta_goal)


if __name__ == '__main__':
    main()

