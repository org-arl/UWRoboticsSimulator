import readchar
import rclpy
from rclpy.node import Node
import os

from std_msgs.msg import String
from geometry_msgs.msg import Pose
pose_msg=Pose()
k=int

pose_msg.position.x=0.0
pose_msg.position.y=0.0
pose_msg.position.z=0.0

pose_msg.orientation.x = 0.0
pose_msg.orientation.y = 0.0
pose_msg.orientation.z = 0.0
pose_msg.orientation.w = 0.0

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('Key_Publisher')
        self.publisher_ = self.create_publisher(Pose, 'key', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        
        
        
        k=ord(repr(readchar.readchar())[1])
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

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
                pose_msg.position.x=0.0
                pose_msg.position.y=0.0
                pose_msg.position.z=0.0

                pose_msg.orientation.x = 00.0
                pose_msg.orientation.y = 0.0
                pose_msg.orientation.z = 0.0
                pose_msg.orientation.w = 0.0

                
        
        if pose_msg.orientation.w==1:
                pose_msg.orientation.w=0.0
        if k==114:
                pose_msg.orientation.w=1.0

        if k==99:
                pose_msg.position.x=0.0
                pose_msg.position.y=0.0
                pose_msg.position.z=0.0

                pose_msg.orientation.x = 0.0
                pose_msg.orientation.y = 0.0
                pose_msg.orientation.z = 0.0
                pose_msg.orientation.w = 0.0
                #pub.publish(pose_msg)
                #break
        
        if k==109:
                pose_msg.orientation.w = 6.0

        if k==91:
                pose_msg.orientation.w = 2.0
        if k==93:
                pose_msg.orientation.w = 3.0

        if k==59:
                pose_msg.orientation.w = 4.0
        if k==39:
                pose_msg.orientation.w = 5.0


        self.publisher_.publish(pose_msg)
        #publisher.publish(pose_msg)

        os.system('clear')
        #rospy.loginfo('Envio: %s', variable)
        #rospy.loginfo('SENDING DATA: ')
        print ("Translation Vectors: ")
        print ("    Surge: ", pose_msg.position.x)
        print ("    Sway:  ", pose_msg.position.y)
        print ("    Heave: ", pose_msg.position.z)
        print (" ")
        print ("Rotation Vectors: ")
        print ("    Pitch: ", pose_msg.orientation.x)
        print ("    Roll:  ", pose_msg.orientation.y)
        print ("    Yaw:   ", pose_msg.orientation.z)
        print (" ")
        print ("Button Function: ")
        print ("    Function ID: ", pose_msg.orientation.w)
        print (" ")




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()