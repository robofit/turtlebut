#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace cv;
using namespace std;
using namespace std_msgs;
using namespace visualization_msgs;

#define MAP_OBSTACLE 0
#define MAP_FREE 255
#define MAP_UNKNOWN 127

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Mat area, draw;	//map storage
float robot_x = 0, robot_y = 0, robot_theta = 0;	//robot position and rotation
bool robot_ready = false;	//tells that map and robo position is ready
int width;	//map width
int height;	//map height
float resolution; //map resolution
float originX;	//coords of origin of the map
float originY;  

//visualization marker - for RVIZ and its parameters
visualization_msgs::Marker marker;

tf::TransformListener *tfl;	//provides info about structure of transformations, allows transforming from one frame to another
int last_x = 0, last_y = 0;	//last goal
vector<Point> blacklist;	//structure containing goals that ceannot be reached, removing items from this list is not implemented
int algorithm = 1;		//algorithm 1 = our, ray based algorithm
				//algorithm 0 = failsafe when algorithm 1 fails - take random free point

// Return the rotation in Euler angles
geometry_msgs::Vector3 GetAsEuler(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 vec;

   double squ;
   double sqx;
   double sqy;
   double sqz;

   squ = quat.w * quat.w;
   sqx = quat.x * quat.x;
   sqy = quat.y * quat.y;
   sqz = quat.z * quat.z;

   // Roll
   vec.x = atan2(2 * (quat.y*quat.z + quat.w*quat.x), squ - sqx - sqy + sqz);

   // Pitch
   vec.y = asin(-2 * (quat.x*quat.z - quat.w * quat.y));

   // Yaw
   vec.z = atan2(2 * (quat.x*quat.y + quat.w*quat.z), squ + sqx - sqy - sqz);

   return vec;
}

// Return the rotation in Euler angles
geometry_msgs::Quaternion GetAsQuaternion(geometry_msgs::Vector3 vec)
{
  geometry_msgs::Quaternion quat;

 	float cos_z_2 = cosf(0.5*vec.z);
	float cos_y_2 = cosf(0.5*vec.y);
	float cos_x_2 = cosf(0.5*vec.x);

	float sin_z_2 = sinf(0.5*vec.z);
	float sin_y_2 = sinf(0.5*vec.y);
	float sin_x_2 = sinf(0.5*vec.x);

	// and now compute quaternion
	quat.w = cos_z_2*cos_y_2*cos_x_2 + sin_z_2*sin_y_2*sin_x_2;
	quat.x = cos_z_2*cos_y_2*sin_x_2 - sin_z_2*sin_y_2*cos_x_2;
	quat.y = cos_z_2*sin_y_2*cos_x_2 + sin_z_2*cos_y_2*sin_x_2;
	quat.z = sin_z_2*cos_y_2*cos_x_2 - cos_z_2*sin_y_2*sin_x_2;

   return quat;
}

//this function asks for the map, get its parameters, robot position and stores it for further use
void getMap() {
	//first get the map
	cout << "getting map" << endl;
	nav_msgs::GetMap::Request req;
	nav_msgs::GetMap::Response resp;
	if(ros::service::call("dynamic_map", req, resp))
	{
		//we check out map data
		width 		= resp.map.info.width;	//width of map in pixels
		height 		= resp.map.info.height;	//height of map in pixels
		resolution 	= resp.map.info.resolution;	//map resolution - size of 1 pixel
		originX 	= -resp.map.info.origin.position.x / resolution;	//the map begins in -val, not 0, so to get the position of robot in pixmap we have to get this value
		originY 	= height + resp.map.info.origin.position.y / resolution - 1;

		cout << "W:" << width << " H:" << height << " res: " << resolution << " orgX:" << originX << " orgY:" << originY << endl;

		area = Mat::zeros(height, width, CV_8UC1);
		//here we convert map to OpenCV image
		for(int a = 0; a < height*width; a++)
		{
			char value = resp.map.data[a];

			if( value == -1 )
				area.at<unsigned char>(a / width, a % width) = MAP_UNKNOWN;
			else if( value < 85 )
				area.at<unsigned char>(a / width, a % width) = MAP_FREE;
			else
				area.at<unsigned char>(a / width, a % width) = MAP_OBSTACLE;
		}
		//flip, because of different frames
		flip(area, area, 0);

		robot_ready = true; //"we have the map" flag

		//draw robot to map image
		draw = area.clone();
		cv::circle(draw, Point((int)robot_x, (int)robot_y), 5, Scalar(200), 2, 8, 0);
		/*imshow("win1", draw);
		waitKey(0);*/
		cout << "got map" << endl;

	} else {
		cout << "cant get map" << endl;
	}

	//to get the position of the robot, we create a pose [0, 0, 0] with rotation 0 in robot frame (base_link)
	//and then ask transformListener to transform it to the map frame
	ros::Time now = ros::Time::now();
	geometry_msgs::PoseStamped source;
	source.header.stamp = now;
	source.header.frame_id = "/base_link";	//source frame
	source.pose.position.x = 0;
	source.pose.position.y = 0;
	source.pose.position.z = 0;
	source.pose.orientation.x = 0.0;
	source.pose.orientation.y = 0.0;
	source.pose.orientation.z = 0.0;
	source.pose.orientation.w = 1.0;

	geometry_msgs::PoseStamped mpose;

	std::string target = "/map";	//destination frame
	
	try {

          if (tfl->waitForTransform(target, source.header.frame_id, now, ros::Duration(2.0))) {	//asking for the transformation

            tfl->transformPose(target,source,mpose);	//actual transformation

          } else {

            source.header.stamp = ros::Time(0);		//this part is not necessary, just in case
            tfl->transformPose(target,source,mpose);
            ROS_WARN("Using latest transform available, may be wrong.");

          }
		
	robot_ready = true;
	cout << "x:" << mpose.pose.position.x << " y: " << mpose.pose.position.y << " z:" << mpose.pose.position.z << endl;
	cout << "quat: " << mpose.pose.orientation.x << " " << mpose.pose.orientation.y << " " << mpose.pose.orientation.z << " " << mpose.pose.orientation.w << endl;

	//convert meter to pixels in openCV image
	robot_x = mpose.pose.position.x / resolution + originX;		//get the position of robot in pixels
	robot_y = -mpose.pose.position.y / resolution + originY;
	geometry_msgs::Vector3 vec = GetAsEuler(mpose.pose.orientation);
	robot_theta = vec.z;						//ge the robot orientation
	cout << "first theta: " << vec.x << " " << vec.y << " " << vec.z << endl;

        } catch(tf::TransformException& ex){
         	std::cerr << "Transform error: " << ex.what() << std::endl;
        }
}

//this function processes map and returns actual goal
move_base_msgs::MoveBaseGoal getGoal(float step, int method) 
{
	//imwrite("test.jpg", draw);

	move_base_msgs::MoveBaseGoal goal;

	cout << "finding goal" << endl;
	//now we check lines from robot to space and try to find unknown cells

	// method == 0 is FailSafe goal finding, if our ray casting algorithm fails
	// it searchs map from top left to bottom right for a free pixel neighbouring with unexplored
	if( method == 0 )
	{
	float dist = 10000;
	int dst_x = 0, dst_y = 0;
	for(int a = 11; a < area.rows-11; a++)
	{
		for(int b = 11; b < area.cols-11; b++)
		{
			int value = (int)(area.at<unsigned char>(a, b));
			if(value == MAP_FREE)
			{
				if( (int)(area.at<unsigned char>(a, b)) == MAP_FREE &&
				    ((int)(area.at<unsigned char>(a+5, b)) == MAP_FREE &&
				     (int)(area.at<unsigned char>(a+5, b)) == MAP_FREE &&
				     (int)(area.at<unsigned char>(a, b+5)) == MAP_FREE &&
				     (int)(area.at<unsigned char>(a, b-5)) == MAP_FREE) &&
				    ((int)(area.at<unsigned char>(a+6, b)) == MAP_UNKNOWN ||
				     (int)(area.at<unsigned char>(a-6, b)) == MAP_UNKNOWN ||
				     (int)(area.at<unsigned char>(a, b+6)) == MAP_UNKNOWN ||
				     (int)(area.at<unsigned char>(a, b-6)) == MAP_UNKNOWN))
				{	
					//measure distance and check that goal
					float new_dst = sqrt( (a - robot_y)*(a - robot_y) + (b - robot_x)*(b - robot_x) );
					//distance from last goal
					//float last_dst = sqrt( (a - last_y)*(a - last_y) + (b - last_x)*(b - last_x) );
					if( new_dst < dist && new_dst > 50 )
					{
						bool is_ok = true;
						//compare against blacklist
						for(int c = 0; c < blacklist.size(); c++) {
							float last_dst = sqrt( (a - blacklist[c].y)*(a - blacklist[c].y) + (b - blacklist[c].x)*(b - blacklist[c].x) );
							if(last_dst < 20)
								is_ok = false;
						}

						if(is_ok)
						{
							dist = new_dst;
							dst_x = b;
							dst_y = a;
						}
					}
				}
			}
		}
	}
	//set goal
	//we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "map"; //target frame
	goal.target_pose.header.stamp = ros::Time::now();

	//goal position and rotation
	goal.target_pose.pose.position.x = (dst_x - originX) * resolution ;
	goal.target_pose.pose.position.y = -(dst_y - originY) * resolution;
	goal.target_pose.pose.orientation.w = 1.0;
	last_x = dst_x;
	last_y = dst_y;

	algorithm = 1;
	cout << "goal found - DUMB" << endl;
	return goal;
	} 
	
	//THIS IS OUR ACTUAL EXPLORATION ALGORITHM
	//based on casting rays from robot position, finding the goal on the border of Free/Unexplored
	else {

	//this cycles the rays around robot. beginning on the left of robot, continuing clockwise
	for( float angle = robot_theta - CV_PI/2 + CV_PI/10; angle < robot_theta + CV_PI + CV_PI/2 - CV_PI/10; angle += step )
	{
		float dy = sin(angle);	//find the increment in X axis
		float dx = cos(angle);	//find the increment in Y axis

		//get the value on actual (where robot is standing) pixel
		int value = (int)(area.at<unsigned char>(robot_y, robot_x));

		//now cast ray from robot with direction defined by angle
		for( int a = 0; a < 800; a ++ )
		{
			int x = robot_x + (int)(dx * a);	//new X position
			int y = robot_y + (int)(dy * a);	//new Y position

			// check if point is in the map
			if( x < 0 || x > area.cols-1 || y < 0 || y > area.rows-1 )
				break;

			//get the value from actual pixel
			int actual = (int)(area.at<unsigned char>(y, x));

			//if we hit Obstacle, we know the goal on this ray wont exist
			if( actual == MAP_OBSTACLE )
				break;

			cv::circle(draw, Point(x, y), 1, Scalar(200), 1, 8, 0);
			//check if we have goal (last pixel is free, new is unexplored)
			if( value == MAP_FREE && actual == MAP_UNKNOWN ) 
			{
				goal.target_pose.header.frame_id = "map"; //target frame of the goal
				goal.target_pose.header.stamp = ros::Time::now();

				//goal position and rotation
				x = robot_x + (int)(dx * (a-15));	// set the goal little bit closer to robot, so it is not on the ege, pathplanner would have problem with that
				y = robot_y + (int)(dy * (a-15));
				goal.target_pose.pose.position.x = (x - originX) * resolution;
				goal.target_pose.pose.position.y = -(y - originY) * resolution;

				//compare against blacklist - checks this goal with goals on blacklist (distance 20 pix around)
				bool is_ok = true;
				for(int c = 0; c < blacklist.size(); c++) {
					//cout << "testing " << blacklist[c].x << "  " << blacklist[c].y << endl;
					//cout << "against " << x << "  " << y << endl;
					float last_dst = sqrt( (y - blacklist[c].y)*(y - blacklist[c].y) + (x - blacklist[c].x)*(x - blacklist[c].x) );
					if(last_dst < 20)
					{
						is_ok = false;
						break;
					}
				}
				if(!is_ok)
					break;

				last_x = x;	//for blacklist purposes
				last_y = y;

				//find out the new robot orientation, this may not be correct :) fix as practical excercise :)
				float yaw;
				float dx = x - robot_x;
				float dy = y - robot_y;
				
				if( dx == 0 )
					if( dy > 0 )
						yaw = CV_PI/2;
					else
						yaw = -CV_PI/2;
				else
					yaw = atan(dy / dx);

				geometry_msgs::Vector3 vec;
				vec.x = 0; vec.y = 0; vec.z = yaw;
				goal.target_pose.pose.orientation = GetAsQuaternion(vec);

				//goal.target_pose.pose.orientation.w = 1.0;
				cv::circle(draw, Point(x, y), 3, Scalar(200), 2, 8, 0);
	
				cout << "goal found - OUR" << endl;
				//cout << endl;
				//imshow("win1", draw);
				//waitKey(0);
				return goal;	//here we are returning new goal
			}

			value = actual;
		}
		cout << "cannot find place to go, changing algorithm" << endl;
		algorithm = 0;
	}
	}
}

//this function is called every time a updated map is available
/*void mapCallback( const nav_msgs::OccupancyGridConstPtr &map ) {
	cout << "map updated" << endl;
}*/

int main( int argc, char** argv )
{
	//initialize ROS, create node handle
	ros::init(argc, argv, "exploration");
	ros::NodeHandle n;

	//marker publisher - this will send visualization to RVIZ
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("our_goal", 0);

	//tf listener - source of robot position
	tfl = new tf::TransformListener();

	//gui to display and draw map
	namedWindow("win1", 1);

	//we want to send navigation goals to robots move-base (server)
	//create client
 	MoveBaseClient ac("move_base", true);

	//action server is responsible for sending goals for robot to reach
	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	//this is refresh rate (10 times per second)
	ros::Rate rate(10);

	//set parameters of marker
	marker.header.frame_id = "map";
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.z = 0.2;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.3;
	marker.scale.y = 0.3;
	marker.scale.z = 0.3;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	//main loop of program, everything happens in here
	while( ros::ok() )
	{

		if( robot_ready )
		{
			//if we got map and robot position, we will try to find new goal
			move_base_msgs::MoveBaseGoal goal = getGoal(CV_PI / 40, algorithm);

			//VISUALIZE GOAL
			marker.header.stamp = ros::Time();
			marker.pose.position.x = goal.target_pose.pose.position.x;
			marker.pose.position.y = goal.target_pose.pose.position.y;
			marker_pub.publish(marker);
			cout << "Actual goal: " << goal.target_pose.pose.position.x << " " << goal.target_pose.pose.position.y << endl;

			//sending goal to move base
			if (ac.sendGoalAndWait(goal,ros::Duration(30),ros::Duration(3)) == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Got to goal, getting new");
			} else {
				ROS_INFO("Timeouted, setting goal to blacklist, changing goal");
				cout << "BLACKLIST: " << last_x << " " << last_y << endl;
				blacklist.push_back(Point(last_x, last_y));
			}
			
			//after reaching/not reaching goal, set this flag to get map and robo position again
			robot_ready = false;
		} else {
			getMap();
		}

		cout << "theta:" << robot_theta << endl;
		
		//this tells node to do its stuff :)
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

