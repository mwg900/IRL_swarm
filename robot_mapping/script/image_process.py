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
from collections import deque                               #윤곽선 중점정보 저장용 queue


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
        #SCAN init
        self.F = False
        
        #Image Process init
        self.width = 360
        self.height = 200

        self.range_max = 6.0 #6m
        self.pts = deque(maxlen = 20)
        self.avr = deque(maxlen = 100)
        
        self.template = cv2.imread("/home/moon/pattern3.png",0)
        #self.template = cv2.Canny(self.template, 50, 200)
        (self.tH, self.tW) = self.template.shape[:2]
        
        if self.template.shape[0] is 0:
            raise AssertionError
        
        
        
        
    #IMU 토픽 콜백(지자기값 출력)
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
        self.F = True
        
    def image_process(self):
        while not rospy.is_shutdown():
            if self.F is True:
                ranges = self.scan_msg.ranges   #메세지로부터 거리 정보 배열 복사. 0~359도의 거리정보를 가지는 배열
                #============================================================
                #    Image Process  
                #============================================================
               
                # Initialize ================================================
                
                root_scale = np.zeros((self.height,self.width,3), dtype = "uint8")
                
                # Pixelization =============================================
                #Original scale
                for i in range(len(ranges)):
                    dist = int((ranges[i]/self.range_max)*self.height*2) #max = 6.0
                    if dist is not 0 and dist < 200: # 3미터 안의 물체만 표시
                        dist = self.height - dist    # y축 표시 반전
                        root_scale[dist:dist+1, i] = (255,255,255)
                
                #cv2.imshow("root_scale",root_scale)
                
                # Blurling =================================================
                dst_image = imutils.resize(root_scale, self.width*2,self.height*2)
                blureed_scale_zoom = cv2.GaussianBlur(dst_image, (11, 7), 0)
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
                
                    #if (area < 40 or 200 < area) or (int(y) <=50): #노이즈 제거
                    if (area < 40 or int(y) <=50): #노이즈 제거
                        cv2.drawContours(mask,[cnt],-1,0,-1)  #마스킹. 검정색으로 해당 덩어리 삭제
                    
                    #Contour 크기 측정 조건문====================================================================
                    
                    elif (100 < int(y) and int(y) <= 230 ) and (8 < diameter and diameter < 17)      :# 거리가 100~230, 지름이 8~17일 시 윤곽선 출력
                        cv2.drawContours(dst_image,[cnt],-1,(0,255,0),1)
                        self.pts.appendleft(center)                                 #큐에 center 좌표 저장
                        print (diameter)
                    elif (240 < int(y) and int(y) <= 280) and (15 < diameter and diameter < 24)       :
                        cv2.drawContours(dst_image,[cnt],-1,(0,255,255),1)
                        self.pts.appendleft(center)
                        print (diameter)
                    elif (280 < int(y) and int(y) <= 350) and (20 < diameter and diameter < 47)       :
                        cv2.drawContours(dst_image,[cnt],-1,(255,0,0),1) 
                        self.pts.appendleft(center)
                        print (diameter)
                    elif (350 < int(y) and int(y) <= 400) and (43 < diameter and diameter < 65)       :
                        cv2.drawContours(dst_image,[cnt],-1,(255,0,255),1) 
                        self.pts.appendleft(center)
                        print (diameter)
                        
                    #=================================================================================    
                    else:
                        cv2.drawContours(mask,[cnt],-1,0,-1)                                                                                     #마스킹. 검정색으로 해당 덩어리 삭제
                #cv2.imshow("mask",mask)
                
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
                for scale in np.linspace(0.7, 1.0, 5)[::-1]:                           #30%의 사이즈까지 총 3번 줄이기.
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
                
                position = Slavepos()
                if found[0] >500000:
                    (startX, startY) = ((maxLoc[0] * r), (maxLoc[1] * r))
                    (endX, endY) = (((maxLoc[0] + self.tW) * r), ((maxLoc[1] + self.tH) * r))
                    tmp_X = int((endX + startX)/2)
                    tmp_Y = int(startY-((endY-startY)/2))
                    dist_tmp = 1000; #임의값
                    #queue 최단거리 계산 
                    for (x,y) in self.pts:
                        dist = math.sqrt(pow(tmp_X-x,2)+pow(tmp_Y-y,2))
                        if dist < dist_tmp:
                            self.cen_X = x
                            self.cen_Y = y
                            dist_tmp = dist
                    
                    self.pts.clear()
                    #Slave 로봇 위치 정보 리턴 및 퍼블리시
                    #cv2.circle(dst_image, (self.cen_X, self.cen_Y+5), 1, (0,255,0), 18,-1)                   #원 출력
                    cv2.rectangle(dst_image, (int(startX), int(startY)), (int(endX), int(endY)), (0, 255, 0), 1)    #사각형 출력  
                    
                    i=0
                    while ranges[self.cen_X/2+i] == 0.0:            #dist 값이 0(inf)일 경우 다음 픽셀의 거리 측정      
                        if i<0:
                            i-=1
                        else :
                            i+=1
                            if i is 2:
                                i=-1
                                
                    srange = "%.2f"%(ranges[self.cen_X/2+i]+0.075)+'m'
                    text = "Id:%d"%slave_id, self.cen_X/2, "Y :%d"%self.cen_Y, srange
                    cv2.putText(dst_image, str(text), (self.cen_X,self.cen_Y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255),1)
                    
                    
                    position.m_ang = self.yaw
                    position.s1_dist = ranges[self.cen_X/2+i]+0.075 
                    position.s1_ang = (self.cen_X/2 + position.m_ang) -180
                    
                    
                    #Graph 출력 용 변수들 
                    position.recog_ang = position.s1_ang + 180   #인식된 슬레이브 로봇 각도(여기선 지자기센서가 없으므로 180을 더해줌)
                    position.real_ang = 180             #실제 슬레이브 로봇 각도
                    position.real_dist = 1.0            #실제 슬레이브 로봇 거리
                    position.sig_timing = s1_theta      #노이즈
                    self.pub.publish(position)          # < r_LOS, phi_LOS, theta > 퍼블리시 
                
                 #Magnetic info line draw 
                mag = (self.yaw)*2  #지자기 각도 yaw. 이미지 크기 때문에 마지막에 2를 곱해주어야 함.
                cv2.line(dst_image, (mag, 0), (mag, self.height*2), (152,225,87), 1)
                
                cv2.imshow("dst_image",dst_image)
                cv2.waitKey(1)
            #self.r.sleep()
#메인문 시작        
if __name__ == '__main__':
    main = Listener()              #클래스 시작
    main.image_process()                                                                                                                                                                                                                                                                                                                                                      