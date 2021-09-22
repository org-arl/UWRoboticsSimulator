#!/usr/bin/env python
import rospy
import array 
from std_msgs.msg import String
from misc.msg import floatarray
import numpy as np


stringArray = ""
depth_publish = []
modelTrigger = 0
def callback(data):
    global depth_publish
    global modelTrigger
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    stringArray = data.data
    arrayLength = 0
    max_range = 0
    if modelTrigger == 1 or modelTrigger == 2:
        arrayLength = 513
    if modelTrigger == 3 or modelTrigger == 4:
        arrayLength = 1025

    if modelTrigger == 2 or modelTrigger == 4:
        max_range = 54.9
    if modelTrigger == 1 or modelTrigger == 3:
        max_range = 94.9
    

    depth_publish = array.array('f',(0 for f in range(0,arrayLength)))
    j=0
    i_start=0
    i_end=0
    while j<arrayLength:
        if i_end >= len(stringArray) or i_start >= len(stringArray):
                break
        while stringArray[i_end]!="a":
            
            i_end+=1
        depth_publish[j]= float((float(stringArray[i_start:i_end]))*(1000.0/1000))
        if depth_publish[j] > max_range:
            depth_publish[j] = np.inf
        #print (stringArray[i_start:i_end])
        i_end+=1
        i_start=i_end
        j+=1
    #print (depth_publish)

def callbackModel(msg):
    global modelTrigger
    modelTrigger = int(msg.data)





        
def listener():
    rospy.Subscriber("depthArray", String, callback)

    # spin() simply keeps python from exiting until this node is stopped

def listenerModel():
    rospy.init_node('stringDecoder', anonymous=True)
    rospy.Subscriber("modelTrigger", String, callbackModel)
    
    

def talker():
    global depth_publish
    pub = rospy.Publisher('decodedDepth', floatarray, queue_size=10)
    #rospy.init_node('decodedDepth', anonymous=True)
    rate = rospy.Rate(50) # 1hz
    while not rospy.is_shutdown():
        publishingarray= floatarray(data=depth_publish)
        pub.publish(publishingarray)
        print (publishingarray)
        rate.sleep()

def main():
    global depth_publish
    global modelTrigger
    
    listenerModel()
    listener()
    talker()


    rospy.spin()

if __name__ == '__main__':
    main()
