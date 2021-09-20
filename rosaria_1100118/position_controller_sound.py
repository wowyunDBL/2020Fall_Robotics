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
import Queue

# sound tool
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

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
        self.current_mode = self.ModeControlSeq.idle.value
        self.current_motion_sequence = self.MotionSequence.searching_marker.value
        self.current_changing_dir_sequence = self.ChangDirSequence.initial_turn

        self.robot_x = .0
        self.robot_y = .0
        self.robot_theta = .0
        self.marker_x = .0
        self.marker_y = .0
        self.marker_theta = .0
        self.last_time_marker = .0

        self.angVel_bound = 1.0
        self.linVel_bound = 0.25

        self.lastPosData = Queue.Queue()
        self.lastPosDataLength = 5

        self.goalOne_x = 0.23
        self.goalOne_y = 0
        self.goalOne_theta = 0
        self.goalTwo_x = 0.15
        self.goalTwo_y = 0
        self.goalTwo_theta = 0
        self.goalX_tolerance = 0.05

        self.is_triggered = False
        self.is_trigger_init = False
        self.is_sequence_finished = False
        self.is_odom_received = False
        self.is_marker_pose_received = False

        loop_rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.is_triggered == True:
                self.fnControlNode()
            loop_rate.sleep()
            self.fnPalySound()   ##########################
            print "aaa"     ##################
            rospy.sleep(10)    ########################
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
            self.lastPosData.put(np.array[pose_x, pose_y, pose_theta])
        else:
            self.lastPosData.get()
            self.lastPosData.put(np.array[pose_x, pose_y, pose_theta])

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
        self.marker_y = meanTurnAroundData[1]
        self.marker_theta = meanTurnAroundData[1]

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

        elif self.current_mode == self.ModeControlSeq.fineTuneForward.value:
            print "Tuning Forward..."
            self.fnMoveSlowSec(2, 0.05) 
            self.msg_pub.data = self.ModeControlSeq.fineTuneForward.value
            self.pub_finish.publish(self.msg_pub)
            self.is_triggered = False
            print "Finished Mode: tuning forward"
        
        elif self.current_mode == self.ModeControlSeq.openDoorBackward.value:
            print "open Door Backward..."
            self.fnMoveSlowSec(2, -0.05) 
            self.msg_pub.data = 3
            self.pub_finish.publish(self.msg_pub)
            self.is_triggered = False
            print "Finished Mode: open Door Backward"

        elif self.current_mode == self.ModeControlSeq.pushDoorForward.value:
            print "push Door Forward..."
            self.fnMoveSlowSec(2, 0.05) 
            self.msg_pub.data = 4
            self.pub_finish.publish(self.msg_pub)
            self.is_triggered = False
            print "Finished Mode: push Door Forward"

        elif self.current_mode == self.ModeControlSeq.leaveDoorBackward.value:
            print "leave Door Backward..."
            self.fnMoveSlowSec(3, -0.05) 
            self.fnPalySound()                     ###########################
            self.msg_pub.data = 5
            self.pub_finish.publish(self.msg_pub)
            self.is_triggered = False
            print "Finished Mode: leave Door Backward"

        elif self.current_mode == self.ModeControlSeq.back2origin.value:
            print "back2origin..."
            if self.fnFaceObstacle() is True:
                print "Obstacle!!!!"
                self.fnStop()                  
            elif self.fnMoving(self.goalTwo_x, self.goalTwo_y, self.goalTwo_theta) == True:
                self.msg_pub.data = 6
                self.pub_finish.publish(self.msg_pub)
                self.is_triggered = False
                self.current_mode = self.ModeControlSeq.idle.value
                print "Finished Mode: back2origin"

    def fnPalySound(self):                    ###################
        soundhandle = SoundClient()

        rospy.sleep(1)
        file = "/home/yuqiang/Treasure.wav"
        rospy.loginfo('Playing "%s".' % file)
        volume = 1.0
        soundhandle.playWave(file, volume)
        
        rospy.sleep(3)

    def fnFineTuneForward(self):
        if self.is_trigger_init == False:
            self.is_trigger_init = True
            self.initial_robot_pose_theta = self.robot_theta
            self.initial_robot_pose_x = self.robot_x
            self.initial_robot_pose_y = self.robot_y
        dist_from_start = self.fnCalcDistPoints(self.initial_robot_pose_x, self.robot_x, self.initial_robot_pose_y, self.robot_y)
        desired_dist = 0.1
        remained_dist = desired_dist - dist_from_start
        print "remained_dist:(<0.01) ",remained_dist
        self.fnGoStraight(remained_dist)
        if remained_dist < 0.01:
            self.fnStop()
            self.is_trigger_init == False
            return True
        return False

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
            self.current_motion_sequence = self.MotionSequence.finished.value
            return True
            #rospy.on_shutdown(self.fnShutDown)
        return False

    def fnSeqSearchingGoal(self):
        
        if self.is_marker_pose_received == False:
            
            self.desired_angle_turn = 0.8
            self.fnTurn(self.desired_angle_turn)
        else:
            self.fnStop()
            return True
    
    def fnSeqMovingNearbyMarker1(self): 
        x = self.marker_x
        y = self.marker_y

        x_diff = -self.goalOne_x + x
        y_diff = -self.goalOne_y + y
        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff) - self.marker_theta + np.pi) % (2 * np.pi) - np.pi # [-pi, pi]
        print (x_diff, y_diff, rho)
        if self.fnFaceObstacle() is True:
            print "Obstacle!!!!"
            self.fnStop() 
            return False                 
        elif rho < 0.8:
            self.fnStop()
            return True
        else:
            self.fnTrackPcontrol(rho, alpha)
            return False
 
    def fnSeqChangingDirection(self):
        # self.ChangDirSequence = Enum('ChangDirSequence', 'initial_turn go_straight turn_forward linedUp')
        if self.current_changing_dir_sequence == self.ChangDirSequence.initial_turn:
            
            if self.is_trigger_init == False:
                self.is_trigger_init = True
                '''
                self.initial_robot_pose_theta = self.robot_theta
                self.initial_robot_pose_x = self.robot_x
                self.initial_robot_pose_y = self.robot_y
                '''

                self.initial_marker_pose_theta = self.marker_theta
                self.initial_marker_pose_x = self.marker_x
                self.initial_marker_pose_y = self.marker_y

                if self.initial_marker_pose_y < 0.15:
                    print "no need to change direction.."
                    return True
            
            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (-(np.pi / 2.0) - self.initial_marker_pose_theta) 
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = ((np.pi / 2.0) - self.initial_marker_pose_theta) 
            
            # mean the last Data For SeqChangingDirection
            n = self.lastDataForSeqChangingDirection.qsize()
            lastTurnAroundData2 = Queue.Queue()
            lastTurnAroundData.queue = copy.deepcopy(self.lastDataForSeqChangingDirection.queue)
            temp = []
            for i in range(n):
                temp.append(lastTurnAroundData.get())
            temp_np = np.array(temp)
            meanTurnAroundData = np.mean(temp_np, 0)

            desired_dist = meanTurnAroundData[0]
            desired_dist = meanTurnAroundData[0]


            self.fnTurnSec(2, desired_angle_turn)
            print "desired_angle_turn:(<0.09) ",desired_angle_turn
            # if abs(desired_angle_turn) < 0.09:
            self.fnStop()
            self.current_changing_dir_sequence = self.ChangDirSequence.go_straight.value
            self.is_trigger_init = False
    
        elif self.current_changing_dir_sequence == self.ChangDirSequence.go_straight.value:
            # dist_from_start = self.fnCalcDistPoints(self.initial_robot_pose_x, self.robot_x, self.initial_robot_pose_y, self.robot_y)
            # desired_dist = self.initial_marker_pose_y
            remained_dist = desired_dist #- dist_from_start
            print "remained_dist:(<0.01) ",remained_dist
            self.fnLIinearSec(2, remained_dist)
            self.fnStop()
            self.current_changing_dir_sequence = self.ChangDirSequence.turn_forward.value

        elif self.current_changing_dir_sequence == self.ChangDirSequence.turn_forward.value:
            '''
            if self.is_trigger_init == False:
                self.is_trigger_init = True
                self.initial_robot_pose_theta = self.robot_theta
            '''
            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (np.pi / 2.0) #- (self.robot_theta - self.initial_robot_pose_theta))
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(np.pi / 2.0) #+ (self.robot_theta - self.initial_robot_pose_theta))
            
            print "desired_angle_turn:(<0.05)", desired_angle_turn
            self.fnTurnSec(3, desired_angle_turn)

            # if abs(desired_angle_turn) < 0.05:
            self.fnStop()
            self.current_changing_dir_sequence = self.ChangDirSequence.linedUp.value
            self.is_trigger_init = False
            return True
                
        return False
    
    def fnSeqMovingNearbyMarker2(self, goalX, goalY, goalTheta): 
        x = self.marker_x
        y = self.marker_y

        x_diff = -goalX + x
        y_diff = -goalY + y
        rho = np.hypot(x_diff, y_diff)
        alpha = 0#(np.arctan2(y_diff, x_diff) - self.marker_theta + np.pi) % (2 * np.pi) - np.pi # [-pi, pi]
        beta = (goalTheta - self.marker_theta - alpha + np.pi) % (2 * np.pi) - np.pi # [-pi, pi]
        print "beta, rho", (beta, rho)
        if self.fnFaceObstacle() is True:
            print "Obstacle!!!!"
            self.fnStop()                 
            return False
        elif x_diff < self.goalX_tolerance and np.abs(beta) < 0.02:
            self.fnStop()
            return True
        else:
            self.fnTrackPcontrol(rho, beta)
            return False
    
    def fnCalcDistPoints(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def fnTurnSec(self, sec, angle):
        angVel = angle/sec
        for i in range(int(sec/0.2)):
            self.fnConstAngVel(angVel)
            time.sleep(0.2)
    
    def fnLIinearSec(self, sec, dist):
        linVel = dist/sec
        for i in range(int(sec/0.2)):
            self.fnConstVel(linVel)
            time.sleep(0.2)

    def fnTurn(self, angle):
        Kp = 0.14
        angularVel = angle * Kp

        msgVel = Twist()
        msgVel.linear.x = 0
        msgVel.linear.y = 0
        msgVel.linear.z = 0
        msgVel.angular.x = 0
        msgVel.angular.y = 0
        if np.abs(angularVel) > self.angVel_bound: 
            if angularVel > 0:
                angularVel = self.angVel_bound
            else:
                angularVel = -self.angVel_bound
        msgVel.angular.z = angularVel
        self.pub_cmd.publish(msgVel)
    
    def fnGoStraight(self, dist):
        Kp = 0.14
        linearVel = dist * Kp
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
        msgVel.angular.z = 0
        
        self.pub_cmd.publish(msgVel)

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
        Kp = 0.8
        angularVel = angle * Kp
        if np.abs(angularVel) > self.angVel_bound : 
            if angularVel > 0:
                angularVel = self.angVel_bound
            else:
                angularVel = -self.angVel_bound

        Kp = 0.14
        linearVel = dist * Kp
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
        rospy.loginfo("Shutting down. cmd_vel will be 0")
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