#!/usr/bin/env python

import roslib; roslib.load_manifest('tb_gazebo')
import rospy
from std_msgs.msg import Float64

class TopicForwarder():
    def __init__(self):
        
        rospy.init_node('topic_forwarder', anonymous=True)
   
        # Start controller state subscribers
        rospy.Subscriber('input_topic', Float64, self.msg_callback)
     
        # Start publisher
        self.pub = rospy.Publisher('output_topic', Float64)
   
        rospy.loginfo('Spinning...')
        
        self.msg_received = False
       
        rospy.spin()
           
    def msg_callback(self, msg):
        
        if not self.msg_received:
            
            rospy.loginfo('First command received, republishing')
            self.msg_received = True
        
        msg.data = -1.0 * msg.data
        
        self.pub.publish(msg)
        
if __name__ == '__main__':
    try:
        t = TopicForwarder()
        rospy.spin()
    except rospy.ROSInterruptException: pass
        
        
