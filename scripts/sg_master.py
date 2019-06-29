#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import time
import sys
import std_srvs.srv
import math
import actionlib
import subprocess
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from std_msgs.msg import String, Bool, Int8, Float64


class StoringGroceries:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)#kobukiの前進後進
        self.navigation_memorize_pub = rospy.Publisher('/navigation/memorize_place', String, queue_size = 1)#目的地を記憶
        self.navigation_command_pub = rospy.Publisher('/navigation/move_place', String, queue_size = 1)#ナビゲーション開始の命令
        self.object_recog_req_pub = rospy.Publisher('/object/recog_req', String, queue_size = 1)#searchの開始
        self.object_list_req_pub = rospy.Publisher('/object/list_req', Bool, queue_size = 1)#objectのリストをもらう
        self.object_grasp_req_pub = rospy.Publisher('/object/grasp_req', String, queue_size = 10)#manipulationの開始
        self.object_count_req_pub = rospy.Publisher('/object/count_req', Bool, queue_size = 1)#objectの個数を要求
        self.changing_pose_pub = rospy.Publisher('/arm/changing_pose_req', String, queue_size = 1)#manipulateしたあとの変形
        self.objet_place_req_pub = rospy.Publisher('/object/place_req', Bool, queue_size=1)#objectを置く
        self.m5_pub = rospy.Publisher('/m5_controller/command', Float64, queue_size = 1)
        self.m6_pub = rospy.Publisher('m6_controller/command', Float64, queue_size = 1)

        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.getLaserCB)                                                                          
        self.object_recog_res_sub = rospy.Subscriber('/object/recog_res', Bool, self.getObjectRecognizeResultCB)
        self.object_list_res_sub = rospy.Subscriber('/object/list_res', String, self.getObjectListCB)
        self.object_grasp_res_sub = rospy.Subscriber('/object/grasp_res', Bool, self.getObjectGraspResultCB)
        self.object_count_res_sub = rospy.Subscriber('/object/count_res', Int8, self.getObjectCountCB)
        self.object_place_res_sub = rospy.Subscriber('/object/place_res', Bool, self.getObjectPlaseCB)
        self.navigation_res_sub = rospy.Subscriber('/navigation/result', Bool, self.getNavigationResultCB)


        self.min_laser_dist = 999.9
        self.front_laser_dist = 999.9
        self.object_recog_flg = False
        self.object_list = []
        self.object_list_flg = False
        self.object_grasp_result_flg = False
        self.object_image_generate_result_flg = False
        self.object_num = -1
        self.object_plase_flg = False
        self.navigation_result_flg = False
        self.place_name = String()
        self.m5_angle = Float64()
        self.m6_angle = Float64()
        self.twist_cmd = Twist()
        self.list_req = False

    def getLaserCB(self, laser_scan):
        self.laser_dist = laser_scan.ranges
        self.min_laser_dist = min(laser_scan.ranges[180:540])
        self.front_laser_dist = laser_scan.ranges[359]

    def getObjectCountCB(self, result_msg):
        self.object_num = result_msg.data

    def getObjectListCB(self, result_msg):
        self.object_list = result_msg.data.split(" ")
        self.object_list[-1:] = []
        print self.object_list
        self.object_num = len(self.object_list)
        self.object_list_flg = True

    def getObjectRecognizeResultCB(self, result_msg):
        self.object_recog_flg = result_msg.data

    def getObjectGraspResultCB(self, result_msg):
        self.object_grasp_result_flg = result_msg.data

    def getObjectPlaseCB(self, result_msg):
        self.object_plase_flg = result_msg.data

    def getNavigationResultCB(self, result_msg):
        print result_msg.data
        self.navigation_result_flg = result_msg.data

    def speak(self, sentense):
        voice_cmd = '/usr/bin/picospeaker %s' %sentense
        subprocess.call(voice_cmd.strip().split(' '))

    def setPlace(self, receive_msg): 
        place_name = String()
        place_name = receive_msg
        rospy.loginfo("Memorizing...")
        self.navigation_memorize_pub.publish(place_name)
        rospy.loginfo("Publeshed topic")
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            self.navigation_memorize_pub.publish(place_name)
            rospy.loginfo("Waiting for result")
            time.sleep(2.0)
        self.navigation_result_flg = False
        rospy.loginfo("Memorization complete!")
        self.speak("I remembered the location og the " + place_name)

    def movePlace(self, receive_msg):
        place_name = String()
        place_name = receive_msg
        self.navigation_command_pub.publish(place_name)
        rospy.loginfo("Moving...")
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            rospy.sleep(3.0)
        self.navigation_result_flg = False
        rospy.loginfo("Has arrived!")
    
    def linearControl(self, linear_num):
        self.twist_cmd.linear.x = linear_num
        self.cmd_vel_pub.publish(self.twist_cmd)
        self.twist_cmd.linear.x = 0

    def angularControl(self, angular_num):
        self.twist_cmd.angular.z = angular_num
        self.cmd_vel_pub.publish(self.twist_cmd)
        self.twist_cmd.angular.z = 0

    def motorControl(self, motor_name, value):
        if motor_name == 5:
            self.m5_angle.data = value
            self.m5_pub.publish(self.m5_angle)
        elif motor_name == 6:
            self.m6_angle.data = value
            self.m6_pub.publish(self.m6_angle)

    def inspectCupboard(self):#-------------------------------------------state0 
        print '-'*80
        rospy.loginfo("Start state0")
        self.speak('I start storing groceries')
        self.motorControl(6, 0.6)
        rospy.loginfo("Advancing!")
        while self.front_laser_dist > 0.5 and not rospy.is_shutdown():
            self.linearControl(0.1)
            #rospy.sleep(0.1)
        rospy.sleep(3.0)
        self.setPlace('cupboard')
        rospy.sleep(2.0)
        rospy.loginfo("Reverse!")
        while self.front_laser_dist < 0.7 and not rospy.is_shutdown():
            self.linearControl(-0.1)
            #rospy.sleep(0.1)
        rospy.sleep(3.0)
        self.speak('I reached the cup board')
        self.speak('Can you open the cup boar door?')
        #扉開いたかの判断はとばす
        rospy.sleep(6.0)
        self.speak('Thank you!')
        #棚の物体を認識してクラス分けに活用する？活用できるか？
        self.object_list_req_pub.publish(list_req)
        self.motorControl(6, 0.0)
        rospy.sleep(1.0)
        
        #位置の微調整 机が棚から見た右側にある場合
        #for i in range(4):
        #    self.linearControl(-0.1)
        #    rospy.sleep(0.5)
        #for i in range(5):
        #    self.angularControl(-1.0)
        #    rospy.sleep(0.5)
        rospy.loginfo('Finish the state0')
        self.movePlace('cupboard')
        return 1

    def findTable(self):#---------------------------------------state1
        print '-'*80
        rospy.loginfo("Start state1")
        self.motorControl(6, -0.2)
        rospy.sleep(2.0)
        #self.angularControl(0)
        list_req =  True
        self.object_list_req_pub.publish(list_req)
        rospy.sleep(3.0)
        print 'Object_num :', self.object_num
        count = 0
        while self.object_num < 3 and not rospy.is_shutdown():
           count += 1
           print 'count'
           self.object_list = []
           rospy.sleep(2.0)
           self.object_list_req_pub.publish(list_req)
           rospy.sleep(2.0)
           print 'Object_num :', self.object_num
           print 'Waiting For Object Recognize!'
           #if count >= 15:
               #count = 0
               #self.m6_angle.data = -0.4
               #self.twist_cmd.angular.z *= -1 #処理を理解する
               #self.motorControl(6, 0.6)
               #self.m6_pub.publish(self.m6_angle.data)
               #rospy.sleep(3.0)
        self.object_num = -1
        #if self.m6_angle.data == -0.4:
            #for i in range(4):
            #    self.linearControl(-0.2)
            #    rospy.sleep(0.5)
            #return 1
            #for i in range(2):
            #    self.linearControl(0.2)
            #    rospy.sleep(0.5)
        self.setPlace('table')
        self.speak('I found the table')
        rospy.loginfo("Finish the state1")
        return 3
    
    def approachingTable(self):#--------------------------------state2
        print '-'*80
        rospy.loginfo("Start state2")
        movePlace('table')
        count = 3
        list_req = True
        self.object_list_req_pub.publish(list_req)
        rospy.sleep(3.0)
        while self.object_num < 2 and not rospy.is_shutdown():
            count += 1
            print 'count'
            self.object_list = []
            self.angularControl(1.0)
            rospy.sleep(2.0)
            self.object_list_req_pub.publish(list_req)
            rospy.sleep(1.0)
            print 'Object num :', self.object_num
            print 'Looking For Object'
            if count >= 6:
                count = 0
                self.twist_cmd.angular.z *= -1#------------不明点ナンバー１
            self.speak('I found objects.')
            self.object_num = -1
        rosoy.loginfo("Finish the state2")
        return 4

    def graspObject(self):#-------------------------------------state3
        print '-'*80
        rospy.loginfo("Start state3")
        list_req = Bool()
        list_req.data = True
        self.object_list_req_pub.publish(list_req)
        rosoy.sleep(2.0)
        object_name = self.object_list[0]
        self.speak('I grasp the ' + object_name)
        grasp_req = String()
        grasp_req.data = object_name
        self.object_grasp_req_pub.publish(grasp_req)
        print 'Wait Object Recognition'
        while self.object_grasp_result_flg == False and not rospy.is_shutdown():
            rospy.sleep(0.5)
        self.object_grasp_result_flg = False
        self.object_list = []
        pose_req = String()
        pose_req.data = 'carry'
        self.changing_pose_pub.publish(pose_req)
        rospy.sleep(1.0)
        rospy.loginfo("Finish the state3")
        return 4   

    def putObject(self):#---------------------------------------state4
        print '-'*80
        rospy.loginfo("Start state4")
        self.movePlace('cupboard')
        list_req = Bool()
        list_req.data = True
        self.twist_cmd.linear.x = 0
        self.twist_cmd.angular.z = -1.0
        count = 2
        self.object_list_req_pub.publish(list_req)
        rospy.sleep(3.0)
        while self.object_num < 3 and not rospy.is_shutdown():
            count += 1
            print count
            self.object_list =[]
            self.angularControl(-1.0)
            rospy.sleep(2.0)
            self.object_list_req_pub.publish(list_req)
            rospy.sleep(1.0)
            print 'Object num:', self.object_num
            print 'Cupboard searching'
            if count >= 4:
                count = 0
                self.twist_cmd.angular.z *= -1#angular.zに-1を書きたものを
        self.object_list = []
        self.object_num = -1
        while self.front_laser_dist < 0.6 and not rospy.is_shutdown():
            self.linearControl(-0.2)
            rospy.sleep(0.1)
        place_req = Bool()
        place_req.data = True
        self.object_place_req_pub.publish(place_req)
        return 6
        print 'Object Placing'
        while self.object_place_flg == False and not rospy.is_shutdown():
           rospy.sleep(0.5)
        rospy.sleep(3.0)
        self.object_place_flg = False
        while self.front_laser_dist > 0.35 and not rospy.is_shutdown():
            self.linearControl(0.1)
            rospy.sleep(0.1)
        rospy.sleep(1.0)
        self.motorControl(5, -0.4)
        rospy.sleep(1.0)
        while self.front_laser_dist < 0.6 and not rospy.is_shutdown():
            self.linearControl(-0.2)
            rospy.sleep(0.1)
        pose_req = String()
        pose_req.data = 'carry'
        self.changing_pose_pub.publish(pose_req)
        rospy.loginfo("Finish the state4")
        return 2

if __name__ == '__main__':
    rospy.init_node('sg_master', anonymous = True)
    sg = StoringGroceries()
    state = 0
    while not rospy.is_shutdown():
        if state == 0:
            state = sg.inspectCupboard()#---------棚を認識して目的地リストに追加
        elif state == 1:
            state = sg.findTable()#---------机を認識して目的地リストに追加
        elif state == 2:
            state = sg.approachingTable()#--把持できる位置まで机に近づく
        elif state == 3:
            state = sg.graspObject()#-------オブジェクトを把持して、棚に移動
        elif state == 4:
            state = sg.putObject()#---------オブジェクトを棚に置く
