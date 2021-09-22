#!/usr/bin/env python

import rospy
import tf
import math
import numpy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from misc.msg import floatarray
import numpy as np
from std_msgs.msg import String
import array
import math



Scan_msg = LaserScan()
Scan_msg.header.frame_id = "base_scan"
#Scan_msg.angle_min = math.pi/4 - 0.05
#Scan_msg.angle_max = math.pi/4 + 0.05
#Scan_msg.angle_increment = 0.1
Scan_msg.time_increment = 0.0
Scan_msg.scan_time = 0.0
Scan_msg.range_min = 0



def listenerModel():

    rospy.Subscriber("modelTrigger", String, callbackModel)

def callback(data):
    global Scan_msg
    i = 0
    while i<101:
        Scan_msg.ranges = data.data
        i+=1

    
def callbackModel(data):
    global modelTrigger
    modelTrigger = int(data.data)
    if modelTrigger == 1 or modelTrigger == 2:
        Scan_msg.angle_max = math.radians(((85.0/512))*256.0)

        Scan_msg.angle_min = math.radians(-((85.0/512)*256.0))

        Scan_msg.angle_increment = math.radians((85.0/512))

    if modelTrigger == 3 or modelTrigger == 4:
        Scan_msg.angle_max = math.radians(((170.0/1024))*512.0)

        Scan_msg.angle_min = math.radians(-(170.0/1024)*512.0)

        Scan_msg.angle_increment = math.radians(170.0/1024)

    if modelTrigger == 2 or modelTrigger == 4:
        Scan_msg.range_max = 54.9
    if modelTrigger == 1 or modelTrigger == 3:
        Scan_msg.range_max = 94.9
    
    

def callbackFOV(data):
    global Scan_msg

#(120/640)*(math.pi/180)
#(float(data.data)*(math.pi / 180))/640

def listener():
    global Scan_msg
    rospy.init_node('Laser_Scan_Publisher')
    rospy.Subscriber("decodedDepth", floatarray, callback)

def listenerFOV():
    global Scan_msg
    rospy.Subscriber('hFOV', String , callbackFOV)


def talker():
    global Scan_msg
    pub = rospy.Publisher('scan', LaserScan, queue_size=10)
    
    #rospy.init_node('decodedDepth', anonymous=True)
    rate = rospy.Rate(50) 
    while not rospy.is_shutdown():
        
        pub.publish(Scan_msg)
        rate.sleep()
def main():
    global Scan_msg
    listenerModel()
    listener()
    listenerFOV()
    talker()
    rospy.spin()

if __name__ == '__main__':
    main()
