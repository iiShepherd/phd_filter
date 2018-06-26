#include <cmath>
#include <stdint.h>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Camerainfo.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "phd_msgs/BearingArray.h"

// Struct a custom Class

struct var
{
	//BoundingBox Param
	std::string Class;
	double probability;
	int64_t xmin;
	int64_t xmax;
	int64_t ymin;
	int64_t ymax;

	//Camerainfo Param
	std::string child_frame_id;
	double fx;
	double fy;
	double cx;
	double cy;
	double min_rng;
	double max_rng;
	double min_ang;
	double max_ang;
	uint32_t width;

	//ROS
	ros::NodeHandle nh;
	ros::publisher bearing_pub;

	//array num_det
	phd_msgs::BearingArray bearing_array;
	unsigned int num_det;
}param;

	//callback functions
void camera_info_callback(const sensor_msgs::Camerainfo::COnstPtr& msg)
{
	param.child_frame_id = msg->header.frame_id;
	param.width = msg->width;
	//focal length
	param.fx = msg->K[0];
	param.fy = msg->K[4];
	//principle points
	param.cx = msg->K[2];
	param.cy = msg->K[5];
	//min/max angle
	param.min_ang = std::atan((cx - width) / fx);
	param.max_ang = std::atan((cx - 1) / fx);


}
void boundingboxes_callback(const dakrnet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	param.Class = msg->bounding_boxes[0].Class;
	param.probability = msg->bounding_boxes[0].probability;
	param.xmin = msg->bounding_boxes[0].xmin;
	param.ymin = msg->bounding_boxes[0].ymin;
	param.xmax = msg->bounding_boxes[0].xmax;
	param.ymax = msg->bounding_boxes[0].ymax;

	param.num_det = msg->bounding_boxes.size();
	bearing_array.array.resize(num_det);
	for (unsigned int k = 0; k < num_det; ++k)
	{
		double centerx = (xmax + xmin) / 2;
		bearing_array.array[k].bearing = std::atan((param.cx - centerx) / param.fx);
		bearing_array.array[k].min_range = param.min_rng;
		bearing_array.array[k].max_range = param.max_rng;
		bearing_array.array[k].min_bearing = param.min_ang;
		bearing_array.array[k].max_bearing = param.max_ang;
		bearing_array.array[k].Class = param.Class;
		bearing_array.array[k].xmin = param.xmin;
		bearing_array.array[k].xmax = param.xmax;
		bearing_array.array[k].ymin = param.ymin;
		bearing_array.array[k].ymax = param.ymax;
	}

	param.bearing_pub.publish(bearing_array);



}

int main(int argc, char** argv)
{
	//initiattion
	ros::init(argc, argv, "detection_converter");
	param.bearing_pub = param.nh.advertise <phd_msgs::BearingArray>("/bridge/measurement", 100);
	//subscriptions (refer to rostopic list)
	ros::Subscriber camera_sub = param.nh.subscribe("/usb_cam/camera_info", 1, &camera_info_callback);
	ros::Subscriber pose_sub = param.nh.subscribe("/pose", 1, &pose_callback);
	ros::Subscriber det_sub = param.nh.subscribe("/darknet_ros/bounding_boxes", 1, &boundingboxes_callback);

	ros::spinOnce();
	return 0;
}