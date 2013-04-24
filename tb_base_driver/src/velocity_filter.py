#!/usr/bin/env python

import roslib; roslib.load_manifest('tb_base_driver')
import rospy
import sys
import os
from geometry_msgs.msg import Twist
from math import fabs, copysign
import copy
from sensor_msgs.msg import LaserScan
from threading import Lock
from math import pi, cos, atan
from scipy.signal import medfilt

class VelocityFilter():

    def __init__(self):
        
        # TODO check of dt value (should not be too big)
        # TODO add obstacle checking
        
        self.acc_lim_x = rospy.get_param("~acc_lim_x",default=0.3)
        self.acc_lim_th = rospy.get_param("~acc_lim_th",default=0.3)
        
        self.curr_vel_x = 0.0
        self.curr_vel_th = 0.0
        
        self.req_vel_x = 0.0
        self.req_vel_th = 0.0
        
        self.vel_lim_x = rospy.get_param("~vel_lim_x",default=0.5)
        self.vel_lim_th = rospy.get_param("~vel_lim_th",default=0.5)
        
        self.vel_sub = rospy.Subscriber("cmd_vel_in",Twist,self.velCallback)
        self.vel_pub = rospy.Publisher("cmd_vel_out",Twist)
        
        self.laser_sub = rospy.Subscriber("scan",LaserScan,self.laserCallback)
        
        self.vel_to = rospy.get_param("~vel_to",default=0.5)
        
        rospy.Timer(rospy.Duration(0.1), self.timerCallback)
        
        self.last_vel = rospy.Time(0)
        
        self.laser_min_idx = None
        self.laser_max_idx = None
        
        self.robot_width = 0.6
        self.limit_dist_l = 0.15
        self.limit_dist_h = 0.4
        
        self.min_dist_act = None

        self.laser_received = False
        
        rospy.loginfo("Initialized")
        
    def limit(self,val,lim):
        
        if (val > lim):
            
            return lim
            
        if (val < -lim):
        
            return -lim
        
        return val
    
    def laserCallback(self,msg):
        
        #msg = LaserScan(data) 
        
        if self.laser_received is False:
            
            rospy.loginfo("First laser scan received")
            self.laser_received = True
            
            l_fov = msg.angle_max - msg.angle_min
            
            # TODO computed angle from min distance and width of robot ?????!!!!!!!!!!!!!!!!
            #req_angle = (90.0/360)*2*pi # this is the angle we want to consider
            req_angle = atan((self.robot_width/2.0) / self.limit_dist_l)
        
            self.laser_min_idx = int( ((l_fov - req_angle)/2)/msg.angle_increment )
            self.laser_max_idx = int(len(msg.ranges) - self.laser_min_idx)
            
            print "Req. angle: " + str(req_angle/(2*pi)*360)
            
            print len(msg.ranges)
            print self.laser_min_idx
            print self.laser_max_idx
            
            
        if self.req_vel_x <= 0.0:
            
            return
            
        dists = []
        
        half_idx = (self.laser_max_idx + self.laser_min_idx)/2 
        
        for d in range(self.laser_min_idx,self.laser_max_idx):
            
            dist = 0.0
            
            #print d
            
            if d < half_idx:
                
                #if d == (self.laser_min_idx+half_idx-10):
                    
                #    print msg.ranges[d]
                #    print (self.laser_min_idx + half_idx-d)*msg.angle_increment
                #    print cos((self.laser_min_idx + half_idx-d)*msg.angle_increment)
                #    print ""
                    
            
                dist = msg.ranges[d]*cos((half_idx-d)*msg.angle_increment)
                
            elif d == half_idx:
                
                dist = msg.ranges[d]
                
            else:
                
                dist = msg.ranges[d]*cos((d-half_idx)*msg.angle_increment)
            
            if dist != 0.0:
                
                dists.append(dist)
            
        #print dists[len(dists)/2-80]
        #print dists[len(dists)/2]
        #print dists[len(dists)/2+80]
        #print ""
        
        dists_f = list(medfilt(dists,5))
            
        dists_f.sort()
        
        self.min_dist_act = dists_f[0]
        
    
    def timerCallback(self,event):
        
        if event.last_real is None:
            
            rospy.loginfo("Timer triggered")
            
            return
        
        dt = event.current_real.to_sec() - event.last_real.to_sec()
        
        #print dt
        
        # store requested velocities
        x = copy.deepcopy(self.req_vel_x)
        th = copy.deepcopy(self.req_vel_th)
        
        if (rospy.Time.now() - self.last_vel) > rospy.Duration(self.vel_to):
            
            x = 0.0
            th = 0.0
        
        if dt == 0.0:
            
            print "dt err"
            return
        
         # limit velocities to some maximum values
        x = self.limit(x, self.vel_lim_x)
        th = self.limit(th, self.vel_lim_th)     
        
        
        if  (self.min_dist_act is not None) and x > 0:
            
            #print self.min_dist_act
            if (self.min_dist_act <= self.limit_dist_l):
                
                x = 0.0 # STOP!
                
            elif self.min_dist_act > self.limit_dist_l and self.min_dist_act<self.limit_dist_h:
                
                # slow down!
                
                max_vel = ((self.min_dist_act - self.limit_dist_l)/(self.limit_dist_h - self.limit_dist_l))*x
                
                print self.min_dist_act
                print max_vel
                print ""
                
                x = self.limit(x, max_vel)
                

        
        # compute requested acceleration
        req_acc_x = (x - self.curr_vel_x)/dt
        req_acc_th = (th - self.curr_vel_th)/dt
        
        # limit requested acceleration
        if x == 0.0:
        
            req_acc_x = self.limit(req_acc_x, 2*self.acc_lim_x)
            
        else:
            
            req_acc_x = self.limit(req_acc_x, self.acc_lim_x)
        
        
        if th == 0.0:
        
            req_acc_th = self.limit(req_acc_th, 2*self.acc_lim_th)
            
        else:
            
            req_acc_th = self.limit(req_acc_th, self.acc_lim_th)
        
        cmd_vel_safe = Twist()
        
        #compute velocities
        cmd_vel_safe.linear.x = self.curr_vel_x + req_acc_x*dt
        cmd_vel_safe.angular.z = self.curr_vel_th + req_acc_th*dt
        
        self.vel_pub.publish(cmd_vel_safe)       
        
        #print dt    
            
        # save current values
        self.curr_vel_x = cmd_vel_safe.linear.x
        self.curr_vel_th = cmd_vel_safe.angular.z
        
        
        
    def velCallback(self,data):
        
        if self.last_vel == rospy.Time(0):
            
            rospy.loginfo("First velocity command received!")
        
        self.req_vel_x = data.linear.x
        self.req_vel_th = data.angular.z
        self.last_vel = rospy.Time.now()
        


if __name__ == '__main__':
    
  try:
      
    rospy.init_node('btb_velocity_filter')
      
    vf = VelocityFilter()
    
    rospy.spin()
    
    
  except rospy.ROSInterruptException: pass