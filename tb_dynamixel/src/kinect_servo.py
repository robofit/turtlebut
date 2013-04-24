#!/usr/bin/env python
import roslib; roslib.load_manifest('tb_dynamixel')
import rospy

from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from tb_dynamixel.srv import SetAngle


class ServoAngleControl():
    
    def __init__(self,servo_name):
        
        rospy.init_node(servo_name + '_angle_control')
        
        rospy.loginfo("Node for setting position of " + servo_name + "servo")
        
        self.default_angle = 0.0
        self.rate = 10.0
        
        self.command_topic_name = servo_name + '_controller/command'
        self.state_topic_name = servo_name +'_controller/state'
        
        self.ns = rospy.get_namespace()
        
        rospy.loginfo('Waiting for ' + servo_name + ' servo controller (' + self.ns + self.state_topic_name + ')')
  
        rospy.wait_for_message(self.state_topic_name,JointState)
        
        self.pub = rospy.Publisher(self.command_topic_name, Float64)
        
        self.angle = None
        
        self.srv = rospy.Service(servo_name + '_controller/set_angle', SetAngle, self.set_angle_srv)
      
    def set_angle_srv(self,req):
        
        # TODO add check for limits!!!!!!!!! -> read it from robot_description ????
        
        if self.angle is None:
            
            rospy.loginfo('First request received.')
        
        self.angle = req.angle
        
        return True
        
    def spin(self):
        
        r = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            
            if self.angle is not None:
                
                self.pub.publish(self.angle)
                
            else:
                
                self.pub.publish(self.default_angle)
            
            r.sleep()
      
              
if __name__ == '__main__':
    
    try:
        
        s = ServoAngleControl('kinect')
        
        s.spin()
        
    except rospy.ROSInterruptException: pass
