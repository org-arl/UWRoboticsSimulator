#!/usr/bin/env python
import rospy
import array 
from std_msgs.msg import String
from misc.msg import floatarray
import numpy as np
import time


stringArray = ""
para_publish = []
def callback(data):
    global para_publish
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    stringArray = data.data
    arrayLength=9
    para_publish = array.array('f',(0 for f in range(0,arrayLength)))
    j=0
    i_start=0
    i_end=0
    while j<arrayLength:
        if i_end >= len(stringArray) or i_start >= len(stringArray):
                break
        while stringArray[i_end]!="a":
            
            i_end+=1
        para_publish[j]= float(stringArray[i_start:i_end])


        i_end+=1
        i_start=i_end
        j+=1
    #print (depth_publish)



        
def listener():

 
    rospy.init_node('paraPublisher', anonymous=True)

    rospy.Subscriber("stateData", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    

def talker():
    global para_publish
    pub = rospy.Publisher('rovParameters', floatarray, queue_size=10)
    #rospy.init_node('decodedDepth', anonymous=True)
    rate = rospy.Rate(50) # 1hz
    while not rospy.is_shutdown():
        publishingarray= floatarray(data=para_publish)
        pub.publish(publishingarray)
        time.sleep(1)
        print(publishingarray)
        rate.sleep()

def main():
    global para_publish
    listener()
    talker()
    rospy.spin()

if __name__ == '__main__':
    main()
