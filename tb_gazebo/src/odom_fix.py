#!/usr/bin/env python

import roslib; roslib.load_manifest('tb_gazebo')
import rospy
#from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class TopicForwarder():
    def __init__(self):
        
        rospy.init_node('odometry_enhancer', anonymous=True)
   
        # Start controller state subscribers
        rospy.Subscriber('odom_in', Odometry, self.msg_callback)
     
        # Start publisher
        self.pub = rospy.Publisher('odom_out', Odometry)
        
        self.cov = [1e-3, 0, 0, 0, 0, 0,
                    0, 1e-3, 0, 0, 0, 0,
                    0, 0, 1e6, 0, 0, 0,
                    0, 0, 0, 1e6, 0, 0,
                    0, 0, 0, 0, 1e6, 0,
                    0, 0, 0, 0, 0, 1e3]
     
   
        rospy.loginfo('Spinning...')
        
        self.msg_received = False
       
        rospy.spin()
           
    def msg_callback(self, msg):
        
        if not self.msg_received:
            
            rospy.loginfo('First odometry msg received, republishing')
            self.msg_received = True
        
        msg.pose.covariance = self.cov
        msg.twist.covariance = self.cov
        
        self.pub.publish(msg)
        
if __name__ == '__main__':
    try:
        t = TopicForwarder()
        rospy.spin()
    except rospy.ROSInterruptException: pass
        
