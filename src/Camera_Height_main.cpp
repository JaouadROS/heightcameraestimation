/*!
*****************************************************************
*
* \note
* Project name: height camera estimation
* \note
* ROS package name: height_camera_estimation
*
* \author
* Author: Jaouad Hajjami - jaouadhajj@gmail.com
*
* \date Date of creation: 06.06.2016
*	\date Date of edition : 27.06.2016
*
* \brief
* Estimate the height of the depth camera
*****************************************************************/
// ROS
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

// OpenCv
#include <opencv/cv.h>
#include <opencv/highgui.h>

// ROS OpenCv
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

//Camera_Height header
#include "height_camera_estimation/Camera_Height.h"

// point cloud
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>

void imageCallbackdepth(const sensor_msgs::ImageConstPtr& msg)
{
	// convert message from ROS to openCV
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
	}

	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// filter depth image - no negatives or NAN - (uchar) img for visualization
	// depth_clean is the true depth
	cv::Mat depth_clean(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);//True depth
	CameraHeight camh1( "depth Image", depth_clean);

	//For visualization
	cv::Mat img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
	for(size_t i = 0; i < cv_ptr->image.rows; i++)
	{
		float* Di = cv_ptr->image.ptr<float>(i);
		float* Ii = depth_clean.ptr<float>(i);
		char* Ivi = img.ptr<char>(i);

		for(size_t j = 0; j < cv_ptr->image.cols; j++)
		{
			if(Di[j] > 0.0f)
			{
				Ii[j] = Di[j];
				Ivi[j] = (char) (255*((Di[j])/(5.5))); // some suitable values.. => For visualization
			}

			else
			{
				Ii[j] = 0.0f;
				Ivi[j] = 0;
			}
		}
	}

	//Visualization
	cv::imshow("depth Image", img);
	cvWaitKey(3);


}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "height_camera_estimation");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber subkidepth = it.subscribe("/camera/depth/image", 1, imageCallbackdepth);

	ROS_INFO("Estimate the height of the camera");

	ros::spin();
}
