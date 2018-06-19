#include <cmath>
#include <stdint.h>
#include <string>
#include <iostream>

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
uint32_t width = 0;

geometry_msgs::PoseStamped currPose;

//ObjectDetection
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

	Class = msg->bounding_boxes[0].Class;
	probability = msg->bounding_boxes[0].probability;
	xmin = msg->bounding_boxes[0].xmin;
	ymin = msg->bounding_boxes[0].ymin;
	xmax = msg->bounding_boxes[0].xmax;
	ymax = msg->bounding_boxes[0].ymax;



}



//End of Callback functions 

//=============================================================================\\

//main function

int main(int argc, char **argv)
{
	ros::init(argc, argv, "detection_converter");
	ros::NodeHandle nh;

	ros::Subscriber camera_sub = nh.subscribe("/usb_cam/camera_info", 1, &camera_info_Callback);
	ros::Subscriber det_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1, &Boundingboxes_Callback);

	ROS_INFO("Converting object detections to bearings...");

	while(ros::ok)
	{
	phd_msgs::BearingArray  bearing_array;
	std::string child_frame_id;
	bearing_array.child_frame_id = child_frame_id;
	bearing_array.pose = currPose;
	
	bearing_array.array.resize(1);									// array  needs to be dynamic
	bearing_array.array[0].min_range = min_rng;
	bearing_array.array[0].max_range = max_rng;
	bearing_array.array[0].min_bearing = min_ang;
	bearing_array.array[0].max_bearing = max_ang;
	bearing_array.array[0].Class = Class;
	bearing_array.array[0].probability = probability;
	bearing_array.array[0].xmin = xmin;
	bearing_array.array[0].xmax = xmax;
	bearing_array.array[0].ymin = ymin;
	bearing_array.array[0].ymax = ymax;

	//Publish Bearing array
	ros::Publisher bearing_pub = nh.advertise<phd_msgs::BearingArray>("/bridge/measurement", 100);

	bearing_pub.publish(bearing_array);
	ros::spinOnce();
	}

 return 0;
}


