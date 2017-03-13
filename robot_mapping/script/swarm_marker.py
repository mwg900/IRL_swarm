#!/usr/bin/env python 
#-*- coding: utf-8 -*- 
import rospy 
from visualization_msgs.msg import Marker
from robot_mapping.msg import Slavepos
#from robot_mapping.srv import Position, PositionResponse 
import math 
 
#클래스 시작 
class robot_marker: 
    def __init__(self): 
        node_name = "swarm_marker_node" 
        rospy.init_node(node_name, anonymous=True) #노드 초기화. 노드명은 1개만 가능 
        position_sub = rospy.Subscriber("/position", Slavepos, self.callback, queue_size= 100)  #포지션 메세지 서브스크라이버 선언
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)    #Marker 메세지 사용하는 viualization_marker 토픽 퍼플리셔 설정  

    def callback(self, msg):
        
        id = 1
        angle = msg.s1_ang+180
        dist = msg.s1_dist
        x = dist*math.cos(math.radians(angle))            #cos 함수가 라디안 값을 받아들임
        y = dist*math.sin(math.radians(angle))

        
        # Renumber the marker IDs
        cylinder = Marker()                 #마커 메세지 사용 
        cylinder.header.frame_id = "imu_link"   #마커를 출력할 프레임 설정 
        cylinder.type = Marker.CYLINDER     #원기둥 타입  
        cylinder.action = Marker.ADD        #마커 추가 동작  
         
        cylinder.id = id                     #고유 id 부여 
         
        cylinder.scale.x = 0.2             #x축으로의 지름 
        cylinder.scale.y = 0.2             #y축으로의 지름 
        cylinder.scale.z = 0.3              #z축으로의 높이 
         
        cylinder.color.g = 1.0              #초록색으로 설정 
        cylinder.color.a = 1.0              #투명도 설정 
         
        cylinder.pose.orientation.w = 1.0 
        cylinder.pose.orientation.w = 1.0           #acos(w)의 값. 마커의 회전을 표시한다 1.0 = 0도  
        cylinder.pose.position.x = x 
        cylinder.pose.position.y = y 
        cylinder.pose.position.z = cylinder.scale.z/2       #맵 상의 평면에 놓기 위해 z축 설정 
             
        self.marker_pub.publish(cylinder)


 
if __name__ == '__main__': 
    try: 
        main = robot_marker()
        rospy.spin()                    #루프 회전 
    except rospy.ROSInterruptException: pass