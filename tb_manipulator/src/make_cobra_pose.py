#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Arizona Robotics Research Group,
# University of Arizona. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import roslib
roslib.load_manifest('tb_manipulator')

import rospy
from std_msgs.msg import Float64
#from pr2_controllers_msgs.msg import JointControllerState
from dynamixel_msgs.msg import JointState

joint_names = ('shoulder_pan_controller',
               'shoulder_pitch_controller',
               'elbow_flex_controller',
               'wrist_roll_controller',
               'left_finger_controller',)
               
joint_commands = (0.0, 1.8, -1.8, 0.0, 0.0)

if __name__ == '__main__':
    pubs = [rospy.Publisher('/dynamixel/' + name + '/command', Float64) for name in joint_names]
    rospy.init_node('make_cobra_pose', anonymous=True)
    
    #rospy.loginfo('Waiting for dynamixel topics to become available...')
    
    #rospy.wait_for_message('/dynamixel/shoulder_pitch_controller/state',JointState)
    
    rospy.loginfo('Publishing positions...')
    
    for n in range(1,10):
        for i in range(len(pubs)):
            pubs[i].publish(joint_commands[i])
        rospy.sleep(0.1)
            
    rospy.loginfo('Ending')
        
