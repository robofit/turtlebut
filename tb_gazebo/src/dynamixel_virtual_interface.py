#!/usr/bin/env python

import roslib; roslib.load_manifest('tb_gazebo')
import rospy
from dynamixel_msgs.msg import JointState
from pr2_controllers_msgs.msg import JointControllerState

#Message for dynamixel controller
#std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
#string name
#int32[] motor_ids
#int32[] motor_temps
#float64 goal_pos
#float64 current_pos
#float64 error
#float64 velocity
#float64 load
#bool is_moving


#message for a PR2 controller
#std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
#float64 set_point
#float64 process_value
#float64 process_value_dot
#float64 error
#float64 time_step
#float64 command
#float64 p
#float64 i
#float64 d
#float64 i_clamp



class TopicForwarder():
    def __init__(self):
        
        rospy.init_node('arm_dynamixel_virtual_interface', anonymous=True)
   
        # Start controller state subscribers
        rospy.Subscriber('shoulder_pan_controller/state_sim', JointControllerState, self.msg_callback_shoulder_pan)
        rospy.Subscriber('shoulder_pitch_controller/state_sim', JointControllerState, self.msg_callback_shoulder_pitch)
        rospy.Subscriber('elbow_flex_controller/state_sim', JointControllerState, self.msg_callback_elbow_flex)
        rospy.Subscriber('wrist_roll_controller/state_sim', JointControllerState, self.msg_callback_wrist_roll)
        rospy.Subscriber('left_finger_controller/state_sim', JointControllerState, self.msg_callback_left_finger)
        rospy.Subscriber('right_finger_controller/state_sim', JointControllerState, self.msg_callback_right_finger)
        rospy.Subscriber('kinect_controller/state_sim', JointControllerState, self.msg_callback_kinect)
     
        # Start publishers
        self.pub_shoulder_pan = rospy.Publisher('shoulder_pan_controller/state', JointState)
        self.pub_shoulder_pitch = rospy.Publisher('shoulder_pitch_controller/state', JointState)
        self.pub_elbow_flex = rospy.Publisher('elbow_flex_controller/state', JointState)
        self.pub_wrist_roll = rospy.Publisher('wrist_roll_controller/state', JointState)
        self.pub_left_finger = rospy.Publisher('left_finger_controller/state', JointState)
        self.pub_right_finger = rospy.Publisher('right_finger_controller/state', JointState)
        self.pub_kinect = rospy.Publisher('kinect_controller/state', JointState)
        
        self.msg_received = False
       
      
    def msg_callback_kinect(self, msg):
               
        out_msg = JointState()
        out_msg.header = msg.header
        out_msg.goal_pos = msg.set_point
        out_msg.current_pos = msg.process_value
        out_msg.error = msg.error
        #out_msg.name = 'kinect_joint'
        #we still need to obtain the speed information about the joint...
        #motor IDs are filled according to a joint of the arm
        out_msg.motor_ids.append(11)
        #other stuff that is not known
        out_msg.load=0
        out_msg.is_moving=0 # we should obtain this information somehow
        out_msg.motor_temps.append(0)
        
        self.pub_kinect.publish(out_msg)
           
    def msg_callback_shoulder_pan(self, msg):
               
        out_msg = JointState()
        out_msg.header = msg.header
        out_msg.goal_pos = msg.set_point
        out_msg.current_pos = msg.process_value
        out_msg.error = msg.error
        #we still need to obtain the speed information about the joint...
        #motor IDs are filled according to a joint of the arm
        out_msg.motor_ids.append(1)
        #other stuff that is not known
        out_msg.load=0
        out_msg.is_moving=0 # we should obtain this information somehow
        out_msg.motor_temps.append(0)
        
        self.pub_shoulder_pan.publish(out_msg)
 
    def msg_callback_shoulder_pitch(self, msg):
        
        out_msg = JointState()
        out_msg.header = msg.header
        out_msg.goal_pos = msg.set_point
        out_msg.current_pos = msg.process_value
        out_msg.error = msg.error
        #we still need to obtain the speed information about the joint...
        #motor IDs are filled according to a joint of the arm
        out_msg.motor_ids.append(2)
        out_msg.motor_ids.append(3)
        #other stuff that is not known
        out_msg.load=0
        out_msg.is_moving=0 # we should obtain this information somehow
        out_msg.motor_temps.append(0)
        out_msg.motor_temps.append(0)
        self.pub_shoulder_pitch.publish(out_msg)
 
    def msg_callback_elbow_flex(self, msg):
        
        out_msg = JointState()
        out_msg.header = msg.header
        out_msg.goal_pos = msg.set_point
        out_msg.current_pos = msg.process_value
        out_msg.error = msg.error
        #we still need to obtain the speed information about the joint...
        #motor IDs are filled according to a joint of the arm
        out_msg.motor_ids.append(4)
        out_msg.motor_ids.append(5)
        #other stuff that is not known
        out_msg.load=0
        out_msg.is_moving=0 # we should obtain this information somehow
        out_msg.motor_temps.append(0)
        out_msg.motor_temps.append(0)
        self.pub_elbow_flex.publish(out_msg)
 
    def msg_callback_wrist_roll(self, msg):
        
        out_msg = JointState()
        out_msg.header = msg.header
        out_msg.goal_pos = msg.set_point
        out_msg.current_pos = msg.process_value
        out_msg.error = msg.error
        #we still need to obtain the speed information about the joint...
        #motor IDs are filled according to a joint of the arm
        out_msg.motor_ids.append(6)
        #other stuff that is not known
        out_msg.load=0
        out_msg.is_moving=0 # we should obtain this information somehow
        out_msg.motor_temps.append(0)
        self.pub_wrist_roll.publish(out_msg)

    def msg_callback_right_finger(self, msg):
        
        out_msg = JointState()
        out_msg.header = msg.header
        out_msg.goal_pos = msg.set_point
        out_msg.current_pos = msg.process_value
        out_msg.error = msg.error
        #we still need to obtain the speed information about the joint...
        #motor IDs are filled according to a joint of the arm
        out_msg.motor_ids.append(7)
        #other stuff that is not known
        out_msg.load=0
        out_msg.is_moving=0 # we should obtain this information somehow
        out_msg.motor_temps.append(0)
        self.pub_right_finger.publish(out_msg)

    def msg_callback_left_finger(self, msg):
        
        out_msg = JointState()
        out_msg.header = msg.header
        out_msg.goal_pos = msg.set_point
        out_msg.current_pos = msg.process_value
        out_msg.error = msg.error
        #we still need to obtain the speed information about the joint...
        #motor IDs are filled according to a joint of the arm
        out_msg.motor_ids.append(8)
        #other stuff that is not known
        out_msg.load=0
        out_msg.is_moving=0 # we should obtain this information somehow
        out_msg.motor_temps.append(0)
        self.pub_left_finger.publish(out_msg)
    
if __name__ == '__main__':
    try:
        t = TopicForwarder()
        rospy.loginfo('Dynamixel virtual interface activated.')
        rospy.spin()
    except rospy.ROSInterruptException: pass
        
        
