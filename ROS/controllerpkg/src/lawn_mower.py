#!/usr/bin/env python
import rospy
import array 
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Float64
import math
import numpy as np
import time
import os
from misc.msg import floatarray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class lawnclass(object):
    def __init__(self):
        self.posub = rospy.Subscriber('/propPos', Pose, self.pose_callback)
        self.depthsub = rospy.Subscriber('/decodedDepth' , floatarray , self.depth_callback)
        self.proxsub = rospy.Subscriber('/proximitySensor', String, self.prox_callback)

        self.conCommand = rospy.Publisher('/key' , Pose , queue_size=10)
        self.auv_pose = Pose()
        self.move_commands = Pose()
        self.distance_holder = Point()
        self.wall_depth = []
        self.start_pose = Pose()
        self.start_pose.position.x = 9.0
        self.start_pose.position.y = -144
        self.start_pose.position.z = 70
        self.start_pose.orientation.x = 0
        self.start_pose.orientation.y = 0
        self.start_pose.orientation.z = 0
        self.start_pose.orientation.w = 1
        self.lawn_mower_flag = 1

    def set_move_commands(self, sent_pose):
        self.move_commands = sent_pose


    def prox_callback(self,msg):
        self.prox_value = float(msg.data)
        
        

    def pose_callback(self,msg):
        self.auv_pose = msg
        self.yaw = euler_from_quaternion([self.auv_pose.orientation.x , self.auv_pose.orientation.y, self.auv_pose.orientation.z, self.auv_pose.orientation.w ])[2]



    def depth_callback(self,data):
        self.wall_depth = [data.data[0], data.data[256], data.data[512]]


    def goToStart(self):
        rate = rospy.Rate(50)
        while math.sqrt(math.pow(self.start_pose.position.x - self.auv_pose.position.x,2) + math.pow(self.start_pose.position.y - self.auv_pose.position.y,2) + math.pow(self.start_pose.position.z - self.auv_pose.position.z,2))>1:
            self.distance_holder.x = self.start_pose.position.x - self.auv_pose.position.x
            self.distance_holder.y = self.start_pose.position.y - self.auv_pose.position.y
            self.distance_holder.z = self.start_pose.position.z - self.auv_pose.position.z            


            self.move_commands.position.x = self.distance_holder.x / math.sqrt(math.pow(self.start_pose.position.x - self.auv_pose.position.x,2) + math.pow(self.start_pose.position.y - self.auv_pose.position.y,2) + math.pow(self.start_pose.position.z - self.auv_pose.position.z,2))
            self.move_commands.position.y = -self.distance_holder.y / math.sqrt(math.pow(self.start_pose.position.x - self.auv_pose.position.x,2) + math.pow(self.start_pose.position.y - self.auv_pose.position.y,2) + math.pow(self.start_pose.position.z - self.auv_pose.position.z,2))
            self.move_commands.position.z = self.distance_holder.z / math.sqrt(math.pow(self.start_pose.position.x - self.auv_pose.position.x,2) + math.pow(self.start_pose.position.y - self.auv_pose.position.y,2) + math.pow(self.start_pose.position.z - self.auv_pose.position.z,2))
            self.conCommand.publish(self.move_commands)
            rate.sleep()

        self.move_commands = Pose()
        self.conCommand.publish(self.move_commands)


    def lawnMower(self):
        
        self.move_commands = Pose()
        rate = rospy.Rate(50)
        while self.wall_depth[0] > 7.0 or self.wall_depth[1] > 7.0 or self.wall_depth[2] > 7.0:
            self.move_commands.position.x = 4
            self.conCommand.publish(self.move_commands)
            rate.sleep()
        self.move_commands = Pose()
        self.conCommand.publish(self.move_commands)

        
        if self.lawn_mower_flag == 1:
            
            while self.yaw>math.radians(-175):
                print (self.yaw)
                self.move_commands.orientation.z = 1
                self.conCommand.publish(self.move_commands)
                rate.sleep()
            self.move_commands = Pose()
            self.conCommand.publish(self.move_commands)

            if self.prox_value < 7:
                self.lawn_mower_flag = 0  #0 Indicates the termination of the program

            else:
                self.lawn_mower_flag = 2
                current_pose = self.auv_pose
                

                while abs(self.auv_pose.position.y - current_pose.position.y) <  + 5.0:
                    print (abs(self.auv_pose.position.y - current_pose.position.y))
                    self.move_commands.position.y = -1
                    self.conCommand.publish(self.move_commands)
                    rate.sleep()

                self.move_commands = Pose()
                self.conCommand.publish(self.move_commands)
            


        elif self.lawn_mower_flag == 2:
            
            if self.prox_value < 7:
                self.lawn_mower_flag = 0  #0 Indicates the termination of the program
            
            else:
                while self.yaw<math.radians(5):
                    print (self.yaw)
                    self.move_commands.orientation.z = -1
                    self.conCommand.publish(self.move_commands)
                    rate.sleep()
                self.move_commands = Pose()
                self.conCommand.publish(self.move_commands)


            
                self.lawn_mower_flag = 1
                current_pose = self.auv_pose
                

                while abs(self.auv_pose.position.y - current_pose.position.y) <  + 5.0:
                    print (abs(self.auv_pose.position.y - current_pose.position.y))
                    self.move_commands.position.y = 1
                    self.conCommand.publish(self.move_commands)
                    rate.sleep()

                self.move_commands = Pose()
                self.conCommand.publish(self.move_commands)






def main():
    rospy.init_node('LawnMower', anonymous=True)
    rate = rospy.Rate(50)

    lawnObject = lawnclass()

    release_trigger = Pose()
    release_trigger.orientation.w = 1
    lawnObject.set_move_commands(release_trigger)
    rospy.sleep(1)

    print ("Releasing ROV")
    lawnObject.conCommand.publish(lawnObject.move_commands)
    rospy.sleep(5)

    print("Sending ROV to Start Position")
    lawnObject.goToStart()
    print ("Start Position Reached")
    rospy.sleep(1)
    print ("Starting Lawn Mower")
    while lawnObject.lawn_mower_flag !=0:
        lawnObject.lawnMower()
        rate.sleep()
    print ("Lawn Mower Ended")
    print ("Exiting Program")
    


if __name__ == '__main__':
    main()