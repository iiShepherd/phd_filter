#include <cmath>
#include <stdint.h>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
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
	ros::Publisher bearing_pub;

	//array num_det
	phd_msgs::BearingArray bearing_array;
	unsigned int num_det;
	
	//pose
	geometry_msgs::PoseStamped currPose;
}param;

	//callback functions
void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{

	
	param.width = msg->width;
	//focal length
	param.fx = msg->K[0];
	param.fy = msg->K[4];
	//principle points
	param.cx = msg->K[2];
	param.cy = msg->K[5];
	//min/max angle
	param.min_ang = std::atan((param.cx - param.width) / param.fx);
	param.max_ang = std::atan((param.cx - 1) / param.fx);


}
void pose_callback ( const nav_msgs::Odometry::ConstPtr& msg)
{
	param.currPose.header = msg->header;
	param.currPose.pose= (msg->pose).pose;

}
void boundingboxes_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	param.num_det = msg->bounding_boxes.size();
	param.bearing_array.array.resize(param.num_det);

	for (unsigned int k = 0; k < param.num_det; ++k)
	{
		param.Class = msg->bounding_boxes[k].Class;
		param.probability = msg->bounding_boxes[k].probability;
		param.xmin = msg->bounding_boxes[k].xmin;
		param.ymin = msg->bounding_boxes[k].ymin;
		param.xmax = msg->bounding_boxes[k].xmax;
		param.ymax = msg->bounding_boxes[k].ymax;

		double centerx = (param.xmax + param.xmin) / 2;
		param.bearing_array.array[k].bearing = std::atan((param.cx - centerx) / param.fx);
		param.bearing_array.array[k].min_range = param.min_rng;
		param.bearing_array.array[k].max_range = param.max_rng;
		param.bearing_array.array[k].min_bearing = param.min_ang;
		param.bearing_array.array[k].max_bearing = param.max_ang;
		param.bearing_array.array[k].Class = param.Class;
		param.bearing_array.array[k].probability=param.probability;
		param.bearing_array.array[k].xmin = param.xmin;
		param.bearing_array.array[k].xmax = param.xmax;
		param.bearing_array.array[k].ymin = param.ymin;
		param.bearing_array.array[k].ymax = param.ymax;
	
		param.bearing_pub.publish(param.bearing_array);
	}




}

int main(int argc, char** argv)
{
	//initiattion
	ros::init(argc, argv, "detection_converter");
	ros::NodeHandle nh;

	//if (nh.getParam("/usb_cam/camera_frame_id", param.child_frame_id) )
	//{
	//ROS_INFO("Got param: %param.child_frame_id", param.child_frame_id.c_str());
	//}
	//else
	//{
	//ROS_ERROR("Failed to get param 'usb_cam/head_camera' ");
	//}


	//subscriptions (refer to rostopic list)
	ros::Subscriber camera_sub = nh.subscribe("/usb_cam/camera_info", 1, &camera_info_callback);
	ros::Subscriber pose_sub = nh.subscribe("/pose", 1, &pose_callback);
	ros::Subscriber det_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1, &boundingboxes_callback);

	param.bearing_pub = nh.advertise <phd_msgs::BearingArray>("/bridge/measurement", 100);
	ROS_INFO("Convering boundingboxes to bearings...");
	ros::spin();
	return 0;
}
