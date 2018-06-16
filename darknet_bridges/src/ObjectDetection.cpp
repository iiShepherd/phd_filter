#include <cmath>
#include <stdint.h>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>

#include "msg/BearingArray.h"
#include "msg/ObjectDetection.h"

//=============================================================================\\

//Beginning of Initializaiton 

//Camera info
double fx = -1;
double fy = -1;
double cx = -1;
double cy = -1;
double min_rng = 0.2;
double max_rng = 15;
double min_ang = 0;
double max_ang = 0;
uint32_t width;

//ObjectDetection
std::string Class;
double score;
uint16_t left;
uint16_t top;
uint16_t right;
uint16_t bottom;

//End of Initializaiton

//=============================================================================\\

//Beginning of Callback functions 

//Calibration Parameters from sensor_msgs/CameraInfo.msg
void camera_info_Callback(const sensor_msgs::CameraInfoConstPtr& msg)
{	
	//Image Dimension width
	width = msg->width;

	//focal lengths 
	fx = msg->k[0];
	fy = msg->k[4];

	//principle points
	cx = msg->k[2];
	cy = msg->k[5];

	//minimum angle
	min_ang = std::atan((cx - width) / fx);
	max_ang = std::atan((cx - 1 / fx));

}

//Object Detection from 

void Object_detection_Callback(const darknet_ros::Boundingbox::ConstPtr& msg)
{
	Class = msg->Class;
	score = msg->bounding_boxes[0].probability;
	left = msg->bounding_boxes[1].left;
	top = msg->bounding_boxes[2].top;
	right = msg->bounding_boxes[3].right;
	bottom = msg->bounding_boxes[4].bottom;

}



//End of Callback functions 

//=============================================================================\\

//main function

int main(int argc, char **argv)
{
	ros::init(argc, argv, "detection_convereter");
	ros::NodeHandle nh;

	ros::subscriber camera_sub = nh.subscribe("camera_info", 1, camera_info_Callback);
	ros::subscriber det_sub = nh.subscribe("Object_detections", 1, Object_detection_Callback);

	//Object Detection Conversion
	darknet_bridge::BearingArray bearing_array;

	bearing_array.array[0].min_range = min_rng;
	bearing_array.array[1].max_range = max_rng;
	bearing_array.array[2].min_bearing = min_ang;
	bearing_array.array[3].max_bearing = max_ang;
	bearing_array.array[4].Class = Class;
	bearing_array.array[5].score = score;
	bearing_array.array[6].left = left;
	bearing_array.array[7].top = top;
	bearing_array.array[8].right = right;
	bearing_array.array[9].bottom = bottom;

	//Publish Bearing array
	Ros::publisher bearing_pub;
	bearing_pub.publish(bearing_array);

	ROS_INFO("Convertin object detections to bearings...");
	ros::spin();
	return 0;

}


