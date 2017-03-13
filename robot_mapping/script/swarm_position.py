#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
import sys
import math

from sensor_msgs.msg import LaserScan
from robot_mapping.msg import Serialmsg       #serial 플래그용 메시지 
from robot_mapping.srv import Position, PositionResponse
import std_msgs

class Listener():
    
    def __init__(self):
        self.node_name = "swarm_position"
        #ros 노드 초기화
        rospy.init_node(self.node_name, anonymous=True)
        #ros topic 구독자 설정 및 콜백함수 정의
        self.laser_sub = rospy.Subscriber("scan", LaserScan, self.callback, queue_size= 100)
        self.zigbee_sub = rospy.Subscriber("slave_sig", Serialmsg, self.zigbee_callback, queue_size= 100)

        self.a = 0
    
    #Laser 토픽 함수
    def callback(self, LaserScan):
        #topic 복사
        self.scan_msg = LaserScan

    
    #slave robot 위치 계산 함수, 각도, 거리 모두 맞지 않음. 확인 필
    def calc_position(self, call_time):
        
        elapsed_time = (call_time - self.scan_msg.header.stamp).to_sec()* 1e-3
        Ka = -539      #-(360+179) 
        angle = ((elapsed_time/self.scan_msg.time_increment) * math.degrees(self.scan_msg.angle_increment) + Ka)*-1 #Ka = 각도 조정용 매직남바 
        count = 179 #매직남바 
        arr_cnt = int(round(angle + count))     #반올림 뒤 인트형으로 변환 
        dist = self.scan_msg.ranges[arr_cnt]
        rospy.loginfo("angle : %s", angle) 
        rospy.loginfo("dist : %s", dist)        #현재 각도, 거리는 정상 출력
        
        if dist > self.scan_msg.range_max: 
            x = y = 0 
            return x, y 
        else:
            x = dist*math.cos(math.radians(angle))            #cos 함수가 라디안 값을 받아들임
            y = dist*math.sin(math.radians(angle))
            return x, y
        
    
    #service client 함수, 해당 함수가 호출되면 swarm_marker node에 서비스를 요청한다. 
    def zigbee_sig(self, id, x, y):
        #service 콜이 들어오면 marker에 id, x,y 값 전달
        rospy.wait_for_service('PositionSig')
        robot_marker = rospy.ServiceProxy('PositionSig',Position) #Position 파일을 이용하여 PositionSig 서비스 요청 
        robot_marker(id, x, y)  
            
    #Zigbee 토픽 함수
    def zigbee_callback(self, msg):
        #Flag 셋일 시 구문 마커 찍음
        if msg.flag.data is True:
            (x, y)= self.calc_position(msg.time.data)
            rospy.loginfo("(x, y) = (%s, %s)",x, y)
            if (x or y) is not 0:       #x, y 가 None이 아닐 경우 서비스 콜
                self.zigbee_sig(msg.id, x, y)
        
#메인문 시작        
if __name__ == '__main__':  

    main = Listener()              #클래스 시작
    rospy.spin()                    #루프 회전
    