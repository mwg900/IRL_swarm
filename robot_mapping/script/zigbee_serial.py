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
        
        
        self.r = rospy.Rate(10) # 10hz
        self.send_msg = "<1,2,3,4,5,6,7>"
    
        self.count = 0
    def callback(self, msg):
        
        #m_lvel = "%0.02f" %(msg.m_lvel)
       # m_avel = "%0.02f" %(msg.m_avel)

        m_lvel = "%04d" %(msg.m_lvel*100)
        m_avel = "%04d" %(msg.m_avel*100)

        m_ang =         "%04d"   %(msg.m_ang)
        s1_dist_mm =    "%04.0f" %(msg.s1_dist*1000)
        s2_dist_mm =    "%04.0f" %(msg.s2_dist*1000)
        #s3_dist_mm =    "%04.0f" %(msg.s3_dist*1000)
        s1_ang =        "%04d"   %(msg.s1_ang)
        s2_ang =        "%04d"   %(msg.s2_ang)
        #s3_ang =        "%03d"   %(msg.s3_ang)
        
        #self.send_msg = "<"+m_ang+","+s1_dist_mm+","+s1_ang+","+s2_dist_mm+","+s2_ang+","+s3_dist_mm+","+s3_ang+">" 
        self.send_msg = "<"+m_ang+","+s1_dist_mm+","+s1_ang+","+s2_dist_mm+","+s2_ang+","+m_lvel+","+m_avel+">" 
        print(self.send_msg)
        self.ser.writelines(self.send_msg)
        
    def talker(self):
        #msg = Serialmsg()
        #msg.flag.data = True
        
        while not rospy.is_shutdown():
            #receive = 1
            receive = self.ser.read()        
            msg = Serialmsg()
            
            if (receive is 1) or (receive is 2) or (receive is 3):
                msg.time.data = rospy.Time.now()
                msg.id = int(receive)   
                self.pub.publish(msg)  
                rospy.loginfo("id:%s received", receive)   
                
                
            #self.r.sleep() 최대한 빠른 속도로 동작하기 위하여 주석처리
if __name__ == '__main__':
    try:
        main = zigbee()
        main.talker()
        self.ser.close()
    except rospy.ROSInterruptException: pass

    
