#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud
from std_msgs.msg import Empty, UInt8

# import math tool
from enum import Enum
import numpy as np
import math
import time
import tf
import csv
import Queue, copy

class MotionController:
    def __init__(self, TaskNumber=0):
        #self.taskNumber = TaskNumber
        if TaskNumber == 0:
            self.ModeControlSeq = Enum('ModeControlSeq', 
                                         'go2goal openDoorBackward \
                                            leaveDoorBackward back2origin finished idle')
        elif TaskNumber == 1:
            self.ModeControlSeq = Enum('ModeControlSeq', 
                                         'go2goal fineTuneForward openDoorBackward \
                                            pushDoorForward leaveDoorBackward back2origin finished idle')

        #self.sub_odom = rospy.Subscriber('/RosAria/odom', Odometry, self.cbOdom_wheel)        ## change to another position topic
        self.sub_marker = rospy.Subscriber('aruco2xyz', PoseStamped, self.cbOdom_marker)        ## change to another position topic
        self.sub_mode_control = rospy.Subscriber('/control_mode', UInt8, self.cbReceiveMode)
        self.pub_cmd = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
        self.pub_finish = rospy.Publisher('/finish_from_car', UInt8, queue_size=10)
        self.msg_pub = UInt8()
        
        self.MotionSequence = Enum('MotionSequence', 'searching_marker moving_nearby1 \
                                                        changing_direction moving_nearby2 stop finished')
        self.ChangDirSequence = Enum('ChangDirSequence', 'initial_turn go_straight turn_forward linedUp')
        self.current_mode = self.ModeControlSeq.go2goal.value
        self.current_motion_sequence = self.MotionSequence.searching_marker.value
        self.current_motion_sequence1 = self.MotionSequence.searching_marker.value
        self.current_changing_dir_sequence = self.ChangDirSequence.initial_turn.value

        self.robot_x = .0
        self.robot_y = .0
        self.robot_theta = .0
        self.marker_x = .0
        self.marker_y = .0
        self.marker_theta = .0
        self.last_time_marker = .0

        self.flag = True
        self.fflag = False

        self.angVel_bound = 1.0
        self.linVel_bound = 0.25
        self.angKp = 0.6
        self.linKp = 0.14
        self.fnMoving2BonusAlphaGain = 0.03
        self.fnMoving2BonusBetaGain = 1.0

        self.yAxisAdjustRange = 0.6
        self.lastPosData = Queue.Queue()
        self.lastPosDataLength = 5

        self.goalOne_x = 0.23
        self.goalOne_y = -0.08
        self.goalOne_theta = 0
        self.goalTwo_x = 0.23
        self.goalTwo_y = 0
        self.goalTwo_theta = 0
        self.goalX_tolerance = 0.02  # change this to set goal diff
        self.beta_tolerance = 0.02

        self.is_triggered = False
        self.is_trigger_init = False
        self.is_sequence_finished = False
        self.is_odom_received = False
        self.is_marker_pose_received = False

        loop_rate = rospy.Rate(20) # 10hz
        while not rospy.is_shutdown():
            if self.is_triggered == True:
                self.fnControlNode()
            loop_rate.sleep()

        rospy.on_shutdown(self.fnShutDown)

    def cbReceiveMode(self, mode_msg):
        rospy.loginfo("Starts the process with %d", mode_msg.data)
        self.current_mode = mode_msg.data
        self.is_triggered = True

    def cbOdom_marker(self, markers_msg):
        if not self.is_marker_pose_received:
            self.is_marker_pose_received = True

        pose_x, pose_y, pose_theta = self.fnGetMarkerPose(markers_msg)
        
        
        # updata last Data of position
        if self.lastPosData.qsize() < self.lastPosDataLength:
            self.lastPosData.put(np.array([pose_x, pose_y, pose_theta]))
        else:
            self.lastPosData.get()
            self.lastPosData.put(np.array([pose_x, pose_y, pose_theta]))

        # mean the last Data of position
        n = self.lastPosData.qsize()
        lastPosData2 = Queue.Queue()
        lastPosData2.queue = copy.deepcopy(self.lastPosData.queue)
        temp = []
        for i in range(n):
            temp.append(lastPosData2.get())
        temp_np = np.array(temp)
        meanPosData = np.mean(temp_np, 0)
        
        # self.marker_x = pose_x
        # self.marker_y = pose_y
        # self.marker_theta = pose_theta
        self.marker_x = pose_x
        self.marker_y = pose_y
        self.marker_theta = meanPosData[2]

        self.last_time_marker = time.time()

    def cbOdom_wheel(self, wheel_msg):
        pose_x, pose_y, pose_theta = self.fnGetWheelPose(wheel_msg)
        self.robot_x = pose_x
        self.robot_y = pose_y
        self.robot_theta = pose_theta

    def fnControlNode(self):
        #self.ModeControlSeq = Enum('ModeControlSeq', 'idle go2goal fineTuneForward openDoorBackward pushDoorForward leaveDoorBackward back2origin finished')

        if self.current_mode == self.ModeControlSeq.go2goal.value:
            print "go2goal..."
            if self.fnMoving(self.goalOne_x, self.goalOne_y, self.goalOne_theta) == True:
                print "Finished Mode: go2goal"
                self.msg_pub.data = 1
                self.pub_finish.publish(self.msg_pub)
                self.is_triggered = False
                self.current_motion_sequence = self.MotionSequence.searching_marker.value ##############3
                self.current_changing_dir_sequence = self.ChangDirSequence.initial_turn.value #############
                self.is_marker_pose_received = False

        elif self.current_mode == self.ModeControlSeq.fineTuneForward.value:
            print "Tuning Forward..."
            self.fnMoveSlowSec(3, 0.05) 
            self.msg_pub.data = 3
            self.pub_finish.publish(self.msg_pub)
            self.is_triggered = False
            print "Finished Mode: tuning forward"
        
        elif self.current_mode == self.ModeControlSeq.openDoorBackward.value:
            print "open Door Backward..."
            self.fnMoveSlowSec(1.5, -0.05) 
            self.msg_pub.data = 5
            self.pub_finish.publish(self.msg_pub)
            self.is_triggered = False
            print "Finished Mode: open Door Backward"

        elif self.current_mode == self.ModeControlSeq.pushDoorForward.value:
            print "push Door Forward..."
            self.fnMoveSlowSec(3, 0.05) 
            for i in range(8):
                self.fnConstAngVel(-0.2)
                rospy.sleep(0.2)
            self.msg_pub.data = 4
            self.pub_finish.publish(self.msg_pub)
            self.is_triggered = False
            print "Finished Mode: push Door Forward"

        elif self.current_mode == self.ModeControlSeq.leaveDoorBackward.value and self.flag:
            print "leave Door Backward..."
            self.fnMoveSlowSec(13, -0.05) 
            self.msg_pub.data = 5
            self.pub_finish.publish(self.msg_pub)
            self.is_triggered = False
            self.flag = False
            self.is_marker_pose_received = False
            
            print "Finished Mode: leave Door Backward"

        elif self.current_mode == self.ModeControlSeq.back2origin.value:
            if self.fflag == False:
                self.is_marker_pose_received = False
                self.fflag = True
            print "back2goal..."
            print "marker: ", self.is_marker_pose_received
            if self.fnMoving1(self.goalTwo_x, self.goalTwo_y, self.goalTwo_theta) == True:               #######################3#
                print "Finished Mode: go2goal"
                self.msg_pub.data = 6
                self.pub_finish.publish(self.msg_pub)
                self.is_triggered = False

    def fnMoveSlowSec(self, sec, vel):
        for i in range(int(sec/0.2)):
            self.fnConstVel(vel)
            time.sleep(0.2)

        self.fnStop()

    def fnFaceObstacle(self):
        if time.time()-self.last_time_marker > 0.3:
            return True
        return False

    def fnGetMarkerPose(self, OdomMsg):
        pose_x = OdomMsg.pose.position.x
        pose_y = OdomMsg.pose.position.y
        quaternion = (
            OdomMsg.pose.orientation.x,
            OdomMsg.pose.orientation.y,
            OdomMsg.pose.orientation.z,
            OdomMsg.pose.orientation.w )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        theta = euler[2] 
        if theta < 0:
            pose_theta = np.pi + theta
        else:
            pose_theta = theta - np.pi
        return pose_x, pose_y, pose_theta
    
    def fnGetWheelPose(self, OdomMsg):
        pose_x = OdomMsg.pose.pose.position.x
        pose_y = OdomMsg.pose.pose.position.y
        quaternion = (
            OdomMsg.pose.pose.orientation.x,
            OdomMsg.pose.pose.orientation.y,
            OdomMsg.pose.pose.orientation.z,
            OdomMsg.pose.pose.orientation.w )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        theta = euler[2] 
        return pose_x, pose_y, theta


    def fnMoving(self, goalX, goalY, goalTheta):
        #if self.modeController == 
        if self.current_motion_sequence == self.MotionSequence.searching_marker.value:
            self.is_sequence_finished = self.fnSeqSearchingGoal()
            
            if self.is_sequence_finished == True:
                print "Finished: searching"
                self.current_motion_sequence = self.MotionSequence.moving_nearby1.value
                self.is_sequence_finished = False

        elif self.current_motion_sequence == self.MotionSequence.moving_nearby1.value:
            print "moving closer1..."
            self.is_sequence_finished = self.fnSeqMovingNearbyMarker1()
            
            
            if self.is_sequence_finished == True:
                print "Finished: closing1"
                self.current_motion_sequence = self.MotionSequence.changing_direction.value
                self.is_sequence_finished = False

        elif self.current_motion_sequence == self.MotionSequence.changing_direction.value:
            print "changing_direction..."
            self.is_sequence_finished = self.fnSeqChangingDirection()
            
            if self.is_sequence_finished == True:
                print "Finished: changing direction"
                self.current_motion_sequence = self.MotionSequence.moving_nearby2.value
                self.is_sequence_finished = False

        elif self.current_motion_sequence == self.MotionSequence.moving_nearby2.value:
            print "moving closer2..."
            self.is_sequence_finished = self.fnSeqMovingNearbyMarker2(goalX, goalY, goalTheta)
            
            if self.is_sequence_finished == True:
                print "Finished: closing2"
                self.current_motion_sequence = self.MotionSequence.stop.value
                self.is_sequence_finished = False

        elif self.current_motion_sequence == self.MotionSequence.stop.value:
            self.fnStop()
            print "Finished: stop"
            self.current_motion_sequence = self.MotionSequence.searching_marker.value
            return True
            #rospy.on_shutdown(self.fnShutDown)
        return False
    
    def fnMoving1(self, goalX, goalY, goalTheta):
        #if self.modeController == 
        if self.current_motion_sequence1 == self.MotionSequence.searching_marker.value:
            self.is_sequence_finished = self.fnSeqSearchingGoal()
            if self.is_sequence_finished == True:
                print "Finished: searching"
                self.current_motion_sequence1 = self.MotionSequence.moving_nearby1.value
                self.is_sequence_finished = False
                self.current_changing_dir_sequence = self.ChangDirSequence.initial_turn.value

        elif self.current_motion_sequence1 == self.MotionSequence.moving_nearby1.value:
            print "moving closer1..."
            self.is_sequence_finished = self.fnSeqMovingNearbyMarker1()
            
            
            if self.is_sequence_finished == True:
                print "Finished: closing1"
                self.current_motion_sequence1 = self.MotionSequence.changing_direction.value
                self.is_sequence_finished = False

        elif self.current_motion_sequence1 == self.MotionSequence.changing_direction.value:
            print "changing_direction..."
            self.is_sequence_finished = self.fnSeqChangingDirection()
            
            if self.is_sequence_finished == True:
                print "Finished: changing direction"
                self.current_motion_sequence1 = self.MotionSequence.moving_nearby2.value
                self.is_sequence_finished = False

        elif self.current_motion_sequence1 == self.MotionSequence.moving_nearby2.value:
            print "moving closer2..."
            self.is_sequence_finished = self.fnSeqMovingNearbyMarker2(goalX, goalY, goalTheta)
            
            if self.is_sequence_finished == True:
                print "Finished: closing2"
                self.current_motion_sequence1 = self.MotionSequence.stop.value
                self.is_sequence_finished = False

        elif self.current_motion_sequence1 == self.MotionSequence.stop.value:
            self.fnTurnSec(4, np.pi)
            self.fnStop()
            print "Finished: stop"
            self.current_motion_sequence1 = self.MotionSequence.searching_marker.value
            return True
            #rospy.on_shutdown(self.fnShutDown)
        return False

    def fnSeqSearchingGoal(self):
        
        if self.is_marker_pose_received == False:
            self.desired_angle_turn = 0.2
            self.fnConstAngVel(self.desired_angle_turn)
        else:
            self.fnStop()
            return True
        return False
    
    def fnSeqMovingNearbyMarker1(self): 
        x = self.marker_x
        y = self.marker_y

        x_diff = -self.goalOne_x + x
        y_diff = -self.goalOne_y + y
        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff) - self.marker_theta + np.pi) % (2 * np.pi) - np.pi # [-pi, pi]
        print "x_diff: %.3f, y_diff: %.3f, rho: %.3f" % (x_diff, y_diff, rho)
        if self.fnFaceObstacle() is True:
            print "Obstacle!!!!"
            self.fnStop() 
            return False                 
        elif rho < self.yAxisAdjustRange:
            self.fnStop()
            return True
        else:
            self.fnTrackPcontrol(rho, alpha)
            return False
 
    def fnSeqChangingDirection(self):
        # self.ChangDirSequence = Enum('ChangDirSequence', 'initial_turn go_straight turn_forward linedUp')
        if self.current_changing_dir_sequence == self.ChangDirSequence.initial_turn.value:
            
            self.initial_marker_pose_theta = self.marker_theta
            self.initial_marker_pose_x = self.marker_x
            self.initial_marker_pose_y = self.marker_y
            print "Initial Marker Y: ", self.initial_marker_pose_y
            if abs(self.initial_marker_pose_y) < 0.15:
                print "no need to change direction..: ", self.initial_marker_pose_y
                return True
            
            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (-(np.pi / 2.0) - self.initial_marker_pose_theta) 
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = ((np.pi / 2.0) - self.initial_marker_pose_theta) 
            
            
            self.fnTurnSec(2, desired_angle_turn)
            print "desired_angle_turn:(<0.09) ",desired_angle_turn
            # if abs(desired_angle_turn) < 0.09:
            self.fnStop()
            self.current_changing_dir_sequence = self.ChangDirSequence.go_straight.value
            self.is_trigger_init = False
    
        elif self.current_changing_dir_sequence == self.ChangDirSequence.go_straight.value:
            # dist_from_start = self.fnCalcDistPoints(self.initial_robot_pose_x, self.robot_x, self.initial_robot_pose_y, self.robot_y)
            desired_dist = self.initial_marker_pose_y
            remained_dist = desired_dist #- dist_from_start
            print "remained_dist:(<0.01) ",remained_dist
            self.fnLIinearSec(2, abs(remained_dist))
            self.fnStop()
            self.current_changing_dir_sequence = self.ChangDirSequence.turn_forward.value

        elif self.current_changing_dir_sequence == self.ChangDirSequence.turn_forward.value:
            '''
            if self.is_trigger_init == False:
                self.is_trigger_init = True
                self.initial_robot_pose_theta = self.robot_theta
            '''
            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (np.pi / 2.2) #- (self.robot_theta - self.initial_robot_pose_theta))
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(np.pi / 2.2) #+ (self.robot_theta - self.initial_robot_pose_theta))
            
            print "desired_angle_turn:(<0.05)", desired_angle_turn
            self.fnTurnSec(3, desired_angle_turn)

            # if abs(desired_angle_turn) < 0.05:
            self.fnStop()
            self.current_changing_dir_sequence = self.ChangDirSequence.initial_turn.value  ################################
            self.is_trigger_init = False
            return True
                
        return False
    
    def fnSeqMovingNearbyMarker2(self, goalX, goalY, goalTheta): 
        x = self.marker_x
        y = self.marker_y

        x_diff = -goalX + x
        y_diff = -goalY + y
        rho = np.hypot(x_diff, y_diff)
        alpha = 0#(np.arctan2(y_diff, x_diff) - self.marker_theta + np.pi) % (2 * np.pi) - np.pi # [-pi, pi]      ################# You need to set 0 to reduce angle
        beta = (goalTheta - self.marker_theta + alpha + np.pi) % (2 * np.pi) - np.pi # [-pi, pi]
        print "beta: %.3f, alpha: %.3f, rho: %.3f" % (beta, alpha, rho)
        if self.fnFaceObstacle() is True:
            print "Obstacle!!!!"
            self.fnStop()                 
            return False
        elif (self.current_mode == self.ModeControlSeq.back2origin.value) and (x_diff < 0.1 and np.abs(beta) < 0.08):
            self.fnStop()
            return True

        elif (self.current_mode == self.ModeControlSeq.go2goal.value) and (x_diff < self.goalX_tolerance and np.abs(beta) < self.beta_tolerance):
            self.fnStop()
            return True
            '''
        elif np.abs(rho) > 0.8 :
            self.fnTrackPcontrol(rho, self.fnMoving2BonusAlphaGain * (1*alpha) ) ################# You need to set 0 to reduce angle
            return False
            '''
        else:
            self.fnTrackPcontrol(rho, self.fnMoving2BonusBetaGain * beta )#+ self.fnMoving2BonusAlphaGain * (-1*alpha) ) ################# You need to set 0 to reduce angle
            return False
    
    def fnTurnSec(self, sec, angle):
        angVel = angle/sec
        for i in range(int(sec/0.2)):
            self.fnConstAngVel(angVel)
            time.sleep(0.2)
    
    def fnLIinearSec(self, sec, dist):
        
        print("dist: %.2f" %dist)
        linVel = dist/sec
        for i in range(int(sec/0.2)):
            self.fnConstVel(linVel)
            time.sleep(0.2)
  
    def fnConstVel(self, vel):
        msgVel = Twist()
        if np.abs(vel) > self.linVel_bound:
            if vel > 0:
                vel = self.linVel_bound
            else: 
                vel = -self.linVel_bound
            print "Input velocity over ", self.linVel_bound
        msgVel.linear.x = vel
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = 0
        self.pub_cmd.publish(msgVel)
    
    def fnConstAngVel(self, angVel):
        msgVel = Twist()
        if np.abs(angVel) > self.angVel_bound:
            if angVel > 0:
                angVel = self.angVel_bound
            else: 
                angVel = -1 * self.angVel_bound
            print "Input angVelocity over ", self.angVel_bound
        msgVel.linear.x = 0
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = angVel
        self.pub_cmd.publish(msgVel)

    def fnStop(self):
        msgVel = Twist()
        msgVel.linear.x = 0
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = 0
        self.pub_cmd.publish(msgVel)

    def fnTrackPcontrol(self, dist, angle):
        
        angularVel = angle * self.angKp
        if np.abs(angularVel) > self.angVel_bound : 
            if angularVel > 0:
                angularVel = self.angVel_bound
            else:
                angularVel = -self.angVel_bound

        
        linearVel = dist * self.linKp
        if np.abs(linearVel) > self.linVel_bound :
            if linearVel > 0 :
                linearVel = self.linVel_bound
            else: 
                linearVel = -self.linVel_bound

        msgVel = Twist()
        msgVel.linear.x = linearVel
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        msgVel.angular.z = angularVel
        self.pub_cmd.publish(msgVel)

    def fnShutDown(self):
        print "Shutting down. cmd_vel will be 0 !!!"
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd.publish(twist) 

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Pioneer_motion_node')
    node = MotionController(1) # 0:water, 1:door
    node.main()