#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
import sys
import math
import cv2
import numpy as np
import imutils
import tf
import copy
import random

from sensor_msgs.msg import LaserScan, Imu                  #스캔 메세지
from geometry_msgs.msg import Quaternion                    #지자기 메세지 
from robot_mapping.msg import Slavepos                     #serial 플래그용 메세지
import std_msgs


class Listener():
    
    def __init__(self):
        node_name = "image_process_node"
        #ros 노드 초기화
        rospy.init_node(node_name)
        #ros topic 구독자 설정 및 콜백함수 정의
        laser_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size= 100)
        imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_callback, queue_size= 100)
        self.pub = rospy.Publisher('/position', Slavepos, queue_size=10)
        
        #IMU init
        self.yaw = 0
        
        #Image Process init
        self.width = 360
        self.height = 200
                
        self.range_max = 4.0 #4m
        
        self.template = cv2.imread("/home/moon/pattern2.png",0)
        #self.template = cv2.Canny(self.template, 50, 200)
        (self.tH, self.tW) = self.template.shape[:2]
        
        if self.template.shape[0] is 0:
            raise AssertionError
        
    #IMU 토픽 콜백
    def imu_callback(self, imu):
        quaternion = (
            imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z,
            imu.orientation.w)
        
        #print(quaternion)
        euler = tf.transformations.euler_from_quaternion(quaternion)    #쿼터니안 - 오일러 변환 
        self.yaw = int(math.degrees(euler[2]))                          #(roll, pitch yaw) 중 yaw만 리턴
        if self.yaw < 0:
            self.yaw = 360+self.yaw         #-179~179의 값을 0~359의 값으로 변경
            
    #Laser 토픽 콜백(실질적인 영상처리 수행)
    def scan_callback(self, LaserScan):
        #topic 복사
        self.scan_msg = LaserScan
        
        ranges = self.scan_msg.ranges   #메세지로부터 거리 정보 배열 복사. 0~359도의 거리정보를 가지는 배열
    #============================================================
    #    Image Process  
    #============================================================
       
    # Initialize ================================================
        
        root_scale = np.zeros((self.height,self.width,3), dtype = "uint8")
        
    # Pixelization =============================================
        #Original scale
        for i in range(len(ranges)):
            dist = self.height - int((ranges[i]/8.0)*self.height*2) #max = 8.0
            if dist is not 0 and dist < 200: # 4미터 안의 물체만 표시
                root_scale[dist:dist+1, i] = (255,255,255)
        
        #cv2.imshow("root_scale",root_scale)
        
    # Blurling =================================================
        dst_image = imutils.resize(root_scale, self.width*2,self.height*2)
        blureed_scale_zoom = cv2.GaussianBlur(dst_image, (11, 5), 0)
        #cv2.imshow("blurred",blureed_scale_zoom)
        
    # Contouring ===============================================
        # find contours in the mask and initialize the current
        imgray = cv2.cvtColor(blureed_scale_zoom,cv2.COLOR_BGR2GRAY)   #1채널 이미지 변환
        ret, thresh = cv2.threshold(imgray,20,255,0)                #2진 이미지 변환
        mask = copy.copy(thresh)                                    #이미지 카피 
        
        # The result "contours" is a Python list, where it contains all objects boundary points as separate lists.
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:] # 3개의 리턴 요소 중 뒤의 2개만 리턴
        
        # a크기 측정 후 필터링
        for h,cnt in enumerate(contours):
            M = cv2.moments(cnt)
            area = M['m00']           #모멘트 성분을 이용하여 영역 검출
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            diameter = 2*radius
            radius=int(radius)              #지름
            
            if (area < 40 or 130 < area) or (int(y) <=200): 
                cv2.drawContours(mask,[cnt],-1,0,-1)  #마스킹. 검정색으로 해당 덩어리 삭제
            
            #Contour 크기 측정 조건문====================================================================
            
            elif (200 < int(y) and int(y) <= 280 ) and (10 < diameter and diameter < 15)        and (160*2 < int(x) and int(x) < 200*2):#높이가 100~200, 지름이 30~40일 시 윤곽선 출력
                cv2.drawContours(dst_image,[cnt],-1,(0,255,0),1) 
                print (diameter)   
            elif (280 < int(y) and int(y) <= 330) and (15 < diameter and diameter < 30)         and (160*2 < int(x) and int(x) < 200*2): #높이가 200~300, 지름이 30~40일 시 윤곽선 출력
                cv2.drawContours(dst_image,[cnt],-1,(0,255,255),1)    
                print (diameter)
            elif (330 < int(y) and int(y) <= 400) and (30 < diameter and diameter < 40)         and (160*2 < int(x) and int(x) < 200*2):      #높이가 300~400이하, 지름이 30~40일 시 윤곽선 출력
                cv2.drawContours(dst_image,[cnt],-1,(255,0,0),1)                                                                    #뒤에 x축 지우기 
                print (diameter)
            else:
                cv2.drawContours(mask,[cnt],-1,0,-1)  #마스킹. 검정색으로 해당 덩어리 삭제
                #cv2.drawContours(dst_image,[cnt],-1,(0,0,255),1)    #윤곽선 출력
                pass
        #cv2.imshow("thresh",thresh)      
        
        
    # Template Matching ============================================#
        # Masking 
        
        #a통신 수신 각도 라인 출력
        slave_id = 1
        if slave_id is 1:
            s1_theta = random.randint(160,200)     #임의값 160~200도 설정
            cv2.line(dst_image, (s1_theta*2, 0), (s1_theta*2, thresh.shape[0]), (255,0,255), 1)
            
            if s1_theta < 30:   #해당 영역만 출력하는 마스크 생성 
                cv2.rectangle(mask,((s1_theta+30)*2,0), ((360+s1_theta-30)*2, mask.shape[0]),(0),-1)
 
            elif s1_theta > 330:
                cv2.rectangle(mask,((s1_theta+30-360)*2,0), ((s1_theta-30)*2, mask.shape[0]),(0),-1)
            
            else:
                cv2.rectangle(mask,(0,0),((s1_theta-30)*2,mask.shape[1]),(0), -1)
                cv2.rectangle(mask,((s1_theta+30)*2 ,0), (mask.shape[1], mask.shape[0]),(0), -1)
        cv2.imshow("mask",mask)
            
        
           
        #Matching 시작
        # load the image image, convert it to grayscale, and detect edges
        #template = cv2.imread("/home/moon/pattern.png")
        found = None
    
        # loop over the scales of the image
        for scale in np.linspace(0.3, 1.0, 5)[::-1]:                           #70%의 사이즈까지 총 3번 줄이기.
            # resize the image according to the scale, and keep track of the ratio of the reizing
            resized = imutils.resize(mask, width = int(mask.shape[1] * scale))
            r = mask.shape[1] / float(resized.shape[1])            #ratio
        
            # if the resized image is smaller than the template, then break from the loop
            if resized.shape[0] < self.tH or resized.shape[1] < self.tW:                      #이미지의 크기보다 작아지면 매칭 종료
                break
        
            # detect edges in the resized, grayscale image and apply template matching to find the template in the image
            edged = resized
            #edged = cv2.Canny(resized, 50, 200)
            result = cv2.matchTemplate(edged, self.template, cv2.TM_CCOEFF)                #본격적인 매칭 시작
            (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)                            #4개의 리턴 값 중 2개만 사용. 각각 맥시멈 스코어, 좌표 리턴
        
            # if we have found a new maximum correlation value, then ipdate the bookkeeping variable
            if found is None or maxVal > found[0]:        #새로운, 더욱 매칭이 잘 맞는 것을 찾으면 업데이트
                found = (maxVal, maxLoc, r)
                
        # unpack the bookkeeping varaible and compute the (x, y) coordinates of the bounding box based on the resized ratio
        (_, maxLoc, r) = found
        #print(found[0])
        if found[0] >500000:
            (startX, startY) = ((maxLoc[0] * r), (maxLoc[1] * r))
            (endX, endY) = (((maxLoc[0] + self.tW) * r), ((maxLoc[1] + self.tH) * r))
            self.cen_X = int((endX + startX)/2)
            self.cen_Y = int(startY-((endY-startY)/2))
             
        
    #Slave 로봇 위치 정보 리턴 및 퍼블리시
        #cv2.circle(dst_image, (self.cen_X, self.cen_Y+5), 1, (0,255,0), 18,-1)                   #원 출력
            cv2.rectangle(dst_image, (int(startX), int(startY)), (int(endX), int(endY)), (0, 255, 0), 1)    #사각형 출력  
        i=0
        #print(ranges[cen_X/2+i])
        while ranges[self.cen_X/2+i] == 0.0:
            if i<0:
                i-=1
            else :
                i+=1
                if i is 2:
                    i=-1

        srange = "%.2f"%ranges[self.cen_X/2+i]+'m'
        text = "Id:%d"%slave_id, self.cen_X/2, "Y :%d"%self.cen_Y, srange
        cv2.putText(dst_image, str(text), (self.cen_X,self.cen_Y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255),1)
        
        position = Slavepos()
        position.m_ang = self.yaw
        position.s1_dist = ranges[self.cen_X/2+i] 
        position.s1_ang = (self.cen_X/2 + position.m_ang) -180
        
        self.pub.publish(position)          # < r_LOS, phi_LOS, theta > 퍼블리시 
        
    #Magnetic info line draw 
        mag = (self.yaw)*2  #지자기 각도 yaw. 이미지 크기 때문에 마지막에 2를 곱해주어야 함.
        cv2.line(dst_image, (mag, 0), (mag, self.height*2), (152,225,87), 1)
        
        cv2.imshow("dst_image",dst_image)
        cv2.waitKey(1)
    
#메인문 시작        
if __name__ == '__main__':
    main = Listener()              #클래스 시작
    rospy.spin()                    #루프 회전                                                                                                                                                                                                                                                                                                                                                                 