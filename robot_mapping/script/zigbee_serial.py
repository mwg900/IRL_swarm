#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
import serial
#import time
from std_msgs.msg import Bool, Time
from robot_mapping.msg import Serialmsg, Slavepos

class zigbee:
    def __init__(self):
        rospy.init_node('zigbee_serial')
        position_sub = rospy.Subscriber("/position", Slavepos, self.callback, queue_size= 100)
        self.pub = rospy.Publisher('/Serial', Serialmsg, queue_size=10)
        self.ser = serial.Serial('/dev/Zigbee', 115200) #시리얼 통신용 변수. 115200의 baudrate를 가짐
        
        
        self.r = rospy.Rate(5) # 10hz
        self.send_msg = "<1,2,3,4,5,6,7>"
    
    def callback(self, msg):
        
        s1_dist_mm = "%.0f" %(msg.s1_dist*1000)
        self.send_msg = "<"+str(msg.m_ang)+","+s1_dist_mm+","+str(msg.s1_ang)+","+"0,0,0,0"+">"        
        self.ser.writelines(self.send_msg)
        print(self.send_msg)
        #rospy.loginfo("<m_w,s1_d,s1_w,s2_d,s2_w,s3_d,s3_w> = %s" ,self.send_msg)
    
    def talker(self):
        #msg = Serialmsg()
        #msg.flag.data = True
        
        while not rospy.is_shutdown():
            
            #receive = ser.read()
            receive = 1
            #rospy.loginfo("id: %s received", receive)
            msg = Serialmsg()
            if receive is not '':
                msg.time.data = rospy.Time.now()
                msg.id = int(receive)
                self.pub.publish(msg)           
                #Zigbee serial Message 전송
                #send_msg = "return <%d>" %msg.id
                
                #print(send_msg)

            #self.ser.writelines(self.send_msg)
            #self.r.sleep() 최대한 빠른 속도로 동작하기 위하여 주석처리
        self.ser.close()
if __name__ == '__main__':
    try:
        main = zigbee()
        main.talker()
    except rospy.ROSInterruptException: pass

    