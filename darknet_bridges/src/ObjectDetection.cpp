#include <cmath>
#include <stdint.h>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "phd_msgs/BearingArray.h"


//=============================================================================\\

//Beginning of Initializaiton 

//Camera info
double fx=-1;
double fy=-1;
double cx=-1;
double cy=-1;
double min_rng = 0.2;
double max_rng = 15;
double min_ang = 0;
double max_ang = 0;
uint32_t width;

//ObjectDetection
phd_msgs::BearingArray  bearing_array;
std::string Class;
double probability = 0;
int64_t xmin=0;
int64_t ymin=0;
int64_t xmax=0;
int64_t ymax=0;




//End of Initializaiton

//=============================================================================\\

//Beginning of Callback functions 

//Calibration Parameters from sensor_msgs/CameraInfo.msg
void camera_info_Callback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{	
	//Image Dimension width
	width = msg->width;

	//focal lengths 
	fx = msg->K[0];
	fy = msg->K[4];

	//principle points
	cx = msg->K[2];
	cy = msg->K[5];

	//minimum angle
	min_ang = std::atan((cx - width) / fx);
	max_ang = std::atan((cx - 1 / fx));

}

//Object Detection from 

void Boundingboxes_Callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{

	Class = msg->bounding_boxes[0];
	probability = msg->bounding_boxes[1];
	xmin = msg->bounding_boxes[2];
	ymin = msg->bounding_boxes[3];
	xmax = msg->bounding_boxes[4];
	ymax = msg->bounding_boxes[5];

}



//End of Callback functions 

//=============================================================================\\

//main function

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "detection_converter");
	ros::NodeHandle nh;

	ros::Subscriber camera_sub = nh.subscribe("/camera_info", 1, &camera_info_Callback);
	ros::Subscriber det_sub = nh.subscribe("/boundingboxes", 1, &Boundingboxes_Callback);


	bearing_array.array[1].min_range = min_rng;
	bearing_array.array[2].max_range = max_rng;
	bearing_array.array[3].min_bearing = min_ang;
	bearing_array.array[4].max_bearing = max_ang;
	bearing_array.array[6].Class = Class;
	bearing_array.array[7].probability = probability;
	bearing_array.array[8].xmin = xmin;
	bearing_array.array[9].xmax = xmax;
	bearing_array.array[10].xmax = xmax;
	bearing_array.array[11].ymax = ymax;





	//Publish Bearing array
	ros::Publisher bearing_pub;
	bearing_pub.publish(bearing_array);

	ROS_INFO("Converting object detections to bearings...");
	ros::spin();
	return 0;

}


