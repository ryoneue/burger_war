#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class RandomBot():
    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('/red_bot/enemy', String, queue_size=30)
        #rospy.Subscriber("/red_bot/image_raw", Image, self.show)
        self.bridge = CvBridge()

        self.enemy = -1 # not detected or lost

    def calcTwist(self):
        value = random.randint(1,1000)
        if value < 250:
            x = 0.2
            th = 0
        elif value < 500:
            x = -0.2
            th = 0
        elif value < 750:
            x = 0
            th = 1
        elif value < 1000:
            x = 0
            th = -1
        else:
            x = 0
            th = 0
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def hough(self, img):
        img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        img = img[...,::-1]
        red,green,blue=cv2.split(img)
        img[blue>50]=0
        img[green>50]=0

        red,green,blue=cv2.split(img)

        img[red>100]=255

        img[img<=100]=0

        if img[img!=0].sum() < 255*5:
            enemy = -1
        else:
            enemy = 1
        
        # img = cv2.medianBlur(img, ksize=5)

        # _,contours, hierarchy = cv2.findContours(cv2.cvtColor(img,cv2.COLOR_RGB2GRAY), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # if len(contours) == 0:
        #     enemy = 0
        # elif cv2.contourArea(contours[0]) > 5:
        #     x,y,w,h = cv2.boundingRect(contours[0])
        #     img = cv2.rectangle(img.copy(), (x,y), (x+w,y+h), (0,255,0),5)
        #     enemy = 1
        # else: enemy = 2
        #print(enemy)
        #cv2.imwrite('out/'+str(enemy)+'_'+str(random.random())+'.png', img)
        print(enemy)
        return enemy
        
    def show(self,img):
        H,W = img.height,img.width
        self.img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        enemy = self.hough(self.img.copy())
        #print(img)
        return enemy
        
    def subscribe(self):
        #r = rospy.Rate(6) # change speed 1fps

        #rospy.init_node('DetectRed', anonymous=True)
        
        
        
        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        while not rospy.is_shutdown():
            #twist = self.calcTwist()
            #print(twist)
            #self.vel_pub.publish(twist)
            enemy = rospy.Subscriber("/red_bot/image_raw", Image, self.hough)
            
            self.vel_pub.publish(enemy)
            rospy.spin()


if __name__ == '__main__':
    rospy.init_node('DetectRed')
    bot = RandomBot('Random')
    bot.subscribe()

