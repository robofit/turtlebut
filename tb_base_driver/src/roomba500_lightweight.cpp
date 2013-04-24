/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita on 07/10/2010
*********************************************************************/
#define NODE_VERSION 2.01

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <boost/assign/list_of.hpp>

#include "sensor_msgs/JointState.h"

#include "tb_base_driver/OpenInterface.h"

#include <string>

std::string port;
irobot::OpenInterface * roomba;
double test_vel;
double test_vel_inc;

double test_odom_th;

bool command_tested = false;
bool roomba_is_down = false;

// read this from parameter server
const double max_lin = 0.5;
const double max_ang = 0.5;

ros::Time latest_command = ros::Time(0);

std::string prefixTopic(std::string prefix, char * name)
{
	std::string topic_name = prefix;
	topic_name.append(name);
	
	return topic_name;
}

double g_lin = 0.0;
double g_ang = 0.0;

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{

	if (!command_tested) {

		ROS_ERROR_ONCE("Test of robot base driver is not completed. Can't move base.");
		return;

	};

	double lin = cmd_vel->linear.x;
	double ang = cmd_vel->angular.z;

	if (lin > max_lin) lin = max_lin;
	if (lin < -max_lin) lin = -max_lin;

	if (ang > max_ang) lin = max_ang;
	if (ang < -max_ang) ang = -max_ang;
	
	//double dt = ros::Time::now().toSec() - latest_command.toSec();

	g_lin = lin;
	g_ang = ang;

	latest_command = ros::Time::now();

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "roomba560_light_node");

	ROS_INFO("Roomba for ROS %.2f", NODE_VERSION);
	
	double last_x, last_y, last_yaw;
	double vel_x, vel_y, vel_yaw;
	double dt;

	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");
	
	private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
	private_nh.param<double>("test_vel", test_vel, 0.1);
	private_nh.param<double>("test_odom_th", test_odom_th, 0.02);

	test_vel_inc = test_vel / 20;

	roomba = new irobot::OpenInterface(port.c_str());
	
	ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 50);
	
	sensor_msgs::JointState js;

  js.name.resize(2);
  js.position.resize(2);
  js.velocity.resize(2);
  js.effort.resize(2);
  
  js.name[0] = "left_wheel_joint";
  js.name[1] = "right_wheel_joint";
  
  js.position[0] = 0.0;
  js.position[1] = 0.0;
	
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 5);
	//tf::TransformBroadcaster odom_broadcaster;
	ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelReceived);

	if( roomba->openSerialPort(true) == 0) ROS_INFO("Connected to Roomba.");
	else
	{
		ROS_FATAL("Could not connect to Roomba.");
		ROS_BREAK();
	}

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	int heartvalue = 0;
	bool inc = true;
	
	bool packets_received = false;
	unsigned int packets_not_received = 0;

	unsigned int test_cnt = 0;

	double test_vel_tmp = 0.0;

	ros::Rate r(20.0);
	while(n.ok())
	{

		current_time = ros::Time::now();
		
		last_x = roomba->odometry_x_;
		last_y = roomba->odometry_y_;
		last_yaw = roomba->odometry_yaw_;
		
		if (!command_tested) {

			if (test_vel_tmp<test_vel) test_vel_tmp += test_vel_inc;

			roomba->drive(test_vel_tmp,0.0);
			roomba->setLeds(0, 0, 0, 0, 255, 255);

			if (roomba->odometry_x_ > test_odom_th) {

				roomba->drive(0.0,0.0);
				command_tested = true;
				ROS_INFO("Test of base driver was successful.");

			} else {

				if (test_cnt++ >= 20) { // 2s

					roomba->powerDown();
					roomba->closeSerialPort();
					roomba_is_down = true;

					ROS_ERROR("Test of base driver failed. Shutting down. Please press red button.");
					ros::Duration dur(10);
					dur.sleep();
					break;

				}


			}

		} else {


			if(inc==true) heartvalue += 20;
			else heartvalue -= 20;

			if(heartvalue>=255) {

				heartvalue = 255;
				inc=false;
			}

			if(heartvalue<=0) {

				heartvalue = 0;
				inc=true;

			}

			roomba->setLeds(0, 0, 0, 0, (unsigned char)heartvalue, 255);

		}



		if( (roomba->getSensorPackets(100) == -1) && (packets_not_received++ >= 5) ) {

			roomba->powerDown();
			roomba->closeSerialPort();
			roomba_is_down = true;

			ROS_ERROR("Could not retrieve sensor packets. Shutting down. Please reset robot.");
			ros::Duration dur(20);

			dur.sleep();

			break;

		} else {

			if (!packets_received) {

				ROS_INFO("We are receiving sensor packets!");

				packets_received = true;

			}

			packets_not_received = 0;

			roomba->calculateOdometry();

		}
		
		dt = (current_time - last_time).toSec();
		
		if (command_tested) roomba->drive(g_lin, g_ang);
		
		vel_x = (roomba->odometry_x_ - last_x)/dt;
		vel_y = (roomba->odometry_y_ - last_y)/dt;
		vel_yaw = (roomba->odometry_yaw_ - last_yaw)/dt;
		
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(roomba->odometry_yaw_);
		
		//first, we'll publish the transform over tf
		/*geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		
		odom_trans.transform.translation.x = roomba->odometry_x_;
		odom_trans.transform.translation.y = roomba->odometry_y_;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;*/
		
		//send the transform
		//odom_broadcaster.sendTransform(odom_trans);
		
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		//odom.header.frame_id = "odom";
		odom.header.frame_id = "odom_combined";
		
		//set the position
		odom.pose.pose.position.x = roomba->odometry_x_;
		odom.pose.pose.position.y = roomba->odometry_y_;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		
		//set the velocity
		//odom.child_frame_id = "base_link";
		odom.child_frame_id = "base_footprint";
		odom.twist.twist.linear.x = vel_x;
		odom.twist.twist.linear.y = vel_y;
		odom.twist.twist.angular.z = vel_yaw;
		
		// ZdenekM -> added covariance, needed for robot_pose_ekf...
		odom.pose.covariance = boost::assign::list_of	(1e-3) (0) (0)  (0)  (0)  (0)
                                                  (0) (1e-3)  (0)  (0)  (0)  (0)
                                                  (0)   (0)  (1e6) (0)  (0)  (0)
                                                  (0)   (0)   (0) (1e6) (0)  (0)
                                                  (0)   (0)   (0)  (0) (1e6) (0)
                                                  (0)   (0)   (0)  (0)  (0)  (1e3) ;

		odom.twist.covariance = boost::assign::list_of	(1e-3) (0) (0)  (0)  (0)  (0)
                                                    (0) (1e-3)  (0)  (0)  (0)  (0)
                                                    (0)   (0)  (1e6) (0)  (0)  (0)
                                                    (0)   (0)   (0) (1e6) (0)  (0)
                                                    (0)   (0)   (0)  (0) (1e6) (0)
                                                    (0)   (0)   (0)  (0)  (0)  (1e3) ;
		
		//publish the message
		if (command_tested) {

			odom_pub.publish(odom);

			// stop if there is no command for more than 0.5s
			 if ((ros::Time::now() - latest_command) > ros::Duration(0.5)) {
			 
			  g_ang = 0.0;
			  g_lin = 0.0;
			  roomba->drive(0.0,0.0);
			  
			  }

		}
		
    js_pub.publish(js);



		ros::spinOnce();
		r.sleep();
	}
	
	roomba->setLeds(0, 0, 0, 0, 0, 0);

	if (!roomba_is_down) {

		roomba->powerDown();
		roomba->closeSerialPort();

	}
}

// EOF
