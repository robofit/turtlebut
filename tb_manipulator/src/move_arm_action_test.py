#!/usr/bin/env python
###############################################################################
# \file
#
# $Id:$
#
# Copyright (C) Brno University of Technology
#
# This file is part of software developed by dcgm-robotics@FIT group.
# 
# Author: Zdenek Materna (imaterna@fit.vutbr.cz)
# Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
# Date: dd/mm/2012
#
# This file is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This file is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with this file.  If not, see <http://www.gnu.org/licenses/>.
#

import roslib; roslib.load_manifest('tb_manipulator')
import rospy
import actionlib

from arm_navigation_msgs.msg import MoveArmAction
from arm_navigation_msgs.msg import MoveArmGoal
from arm_navigation_msgs.msg import PositionConstraint
from arm_navigation_msgs.msg import OrientationConstraint
from arm_navigation_msgs.msg import ArmNavigationErrorCodes

def main():
    
    rospy.init_node('move_arm_action_test')
    rospy.loginfo('Node for testing move_arm action')
    
    client = actionlib.SimpleActionClient('/move_arm',MoveArmAction)
    
    client.wait_for_server()
    
    rospy.loginfo('Server is available, let\'s start')
    
    goal = MoveArmGoal()
    
    goal.motion_plan_request.group_name = 'arm'
    goal.motion_plan_request.num_planning_attempts = 5
    goal.motion_plan_request.planner_id = ''
    goal.planner_service_name = '/ompl_planning/plan_kinematic_path'
    goal.motion_plan_request.allowed_planning_time = rospy.Duration(15.0)
        
    pos_const = PositionConstraint()
    
    pos_const.header.frame_id = 'base_footprint'
    pos_const.link_name = 'arm_wrist_roll_link'
    
    # default position
    # rosrun tf tf_echo base_footprint arm_wrist_roll_link
    #- Translation: [0.454, 0.000, 0.229]
    #- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
    #  in RPY [0.000, 0.000, 0.000]

    
    
    pos_const.position.x =  0.315077617598
    pos_const.position.y =  0.260552844631
    pos_const.position.z = 0.279730321177
    
    pos_const.constraint_region_shape.type = pos_const.constraint_region_shape.BOX
    pos_const.constraint_region_shape.dimensions.append(2*0.02)
    pos_const.constraint_region_shape.dimensions.append(2*0.02)
    pos_const.constraint_region_shape.dimensions.append(2*0.02)
    
    pos_const.constraint_region_orientation.x = 0.0
    pos_const.constraint_region_orientation.y = 0.0
    pos_const.constraint_region_orientation.z = 0.0
    pos_const.constraint_region_orientation.w = 1.0
    
    pos_const.weight = 1.0
    
    or_const = OrientationConstraint()
    
    or_const.header.frame_id = 'base_footprint'
    or_const.link_name = 'arm_wrist_roll_link'
    
    or_const.weight = 1.0
    
    # default orientation
    or_const.orientation.x = -0.212992566922
    or_const.orientation.y = -0.105167499832
    or_const.orientation.z = 0.437894676762
    or_const.orientation.w = 0.867076822132
    
    or_const.absolute_pitch_tolerance = 0.04
    or_const.absolute_roll_tolerance = 0.04
    or_const.absolute_yaw_tolerance = 0.04
    
    # --------------------------------------------------
    
    goal.motion_plan_request.goal_constraints.orientation_constraints.append(or_const)
    goal.motion_plan_request.goal_constraints.position_constraints.append(pos_const)
    
    client.send_goal(goal)
    
    client.wait_for_result()
    
    res = client.get_result()
    
    print "Result"
    print res
    
    
    
if __name__ == '__main__':
  main()
