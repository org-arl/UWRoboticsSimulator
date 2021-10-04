#!/usr/bin/env python


import rospy
import getch
import os
import time
#from std_msgs.msg import Float64 # from package.[msg/srv] import ["msg"/"srv"]
from geometry_msgs.msg import Pose
pose_msg = Pose()

pose_msg.position.x=0
pose_msg.position.y=0
pose_msg.position.z=0

pose_msg.orientation.x = 0
pose_msg.orientation.y = 0
pose_msg.orientation.z = 0
pose_msg.orientation.w = 0

def talker():

    pub = rospy.Publisher('key', Pose, queue_size=10) # TOPIC
    rospy.init_node('key_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz


    while not rospy.is_shutdown():

        k=ord(getch.getch())


        if k==119 and pose_msg.position.x<1:

                pose_msg.position.x = pose_msg.position.x + 1

        if k==115 and pose_msg.position.x>-1:
                pose_msg.position.x = pose_msg.position.x -1


        if k==100 and pose_msg.position.y<1:

                pose_msg.position.y = pose_msg.position.y + 1

        if k==97 and pose_msg.position.y>-1:
                pose_msg.position.y = pose_msg.position.y -1


        if k==32 and pose_msg.position.z<1:

                pose_msg.position.z = pose_msg.position.z + 1

        if k==98 and pose_msg.position.z>-1:
                pose_msg.position.z = pose_msg.position.z -1




        if k==105 and pose_msg.orientation.x<1:
                pose_msg.orientation.x = pose_msg.orientation.x + 1

        if k==107 and pose_msg.orientation.x>-1:
                pose_msg.orientation.x = pose_msg.orientation.x -1


        if k==108 and pose_msg.orientation.y<1:
                pose_msg.orientation.y = pose_msg.orientation.y + 1

        if k==106 and pose_msg.orientation.y>-1:
                pose_msg.orientation.y = pose_msg.orientation.y -1


        if k==101 and pose_msg.orientation.z<1:
                pose_msg.orientation.z = pose_msg.orientation.z + 1

        if k==113 and pose_msg.orientation.z>-1:
                pose_msg.orientation.z = pose_msg.orientation.z -1


        if k==120:
                pose_msg.position.x=0
                pose_msg.position.y=0
                pose_msg.position.z=0

                pose_msg.orientation.x = 0
                pose_msg.orientation.y = 0
                pose_msg.orientation.z = 0
                pose_msg.orientation.w = 0



        if pose_msg.orientation.w==1:
                pose_msg.orientation.w=0
        if k==114:
                pose_msg.orientation.w=1

        if k==99:
                pose_msg.position.x=0
                pose_msg.position.y=0
                pose_msg.position.z=0

                pose_msg.orientation.x = 0
                pose_msg.orientation.y = 0
                pose_msg.orientation.z = 0
                pose_msg.orientation.w = 0
                pub.publish(pose_msg)
                break

        if k==109:
                pose_msg.orientation.w = 6

        if k==91:
                pose_msg.orientation.w = 2
        if k==93:
                pose_msg.orientation.w = 3

        if k==59:
                pose_msg.orientation.w = 4
        if k==39:
                pose_msg.orientation.w = 5



        if k==44:
                if pose_msg.orientation.w == 6 :
                        pose_msg.orientation.w = 7
                else:
                        pose_msg.orientation.w = 6

        if k==46:
                if pose_msg.orientation.w ==8:
                        pose_msg.orientation.w = 7
                else:
                        pose_msg.orientation.w = 8








        os.system('clear')
        #rospy.loginfo('Envio: %s', variable)
        rospy.loginfo('SENDING DATA: ')
        print("Translation Vectors: ")
        print("    Surge: ", pose_msg.position.x)
        print("    Sway:  ", pose_msg.position.y)
        print("    Heave: ", pose_msg.position.z)
        print(" ")
        print("Rotation Vectors: ")
        print("    Pitch: ", pose_msg.orientation.x)
        print("    Roll:  ", pose_msg.orientation.y)
        print("    Yaw:   ", pose_msg.orientation.z)
        print(" ")
        print("Button Function: ")
        print("    Function ID: ", pose_msg.orientation.w)
        print(" ")
        time.sleep(1)
        pub.publish(pose_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
