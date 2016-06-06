/*!
*****************************************************************
*
* \note
* Project name: height camera estimation
* \note
* ROS package name: heightcameraestimation
*
* \author
* Author: Jaouad Hajjami - jaouadhajj@gmail.com
*
* \date Date of creation: 06.06.2016
*
* \brief
* Estimate the heigh of the depth camera
*****************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// point cloud
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
/*
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
*/

cv::Mat depthimagemouse;

size_t i_mouse=0; //

const float //dc1  = -0.0030711016,
						//dc2  = 3.3309495161,
						fx_d = 5.9421434211923247e+02,
						fy_d = -5.9104053696870778e+02,
						px_d = 320,
						py_d = 240;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pointsplane(new pcl::PointCloud<pcl::PointXYZ>);

void onMouse( int event, int x, int y, int, void* )
{
    if( event != CV_EVENT_LBUTTONDOWN )
            return;

    cv::Point pt = cv::Point(x,y);

		float z=depthimagemouse.at<float>(y,x);
		//std::cout<<"Halloo"<<std::endl;

		cloud_pointsplane->width = 10; //plane segmentation of a set of 10 points
		cloud_pointsplane->height = 1;
		cloud_pointsplane->points.resize(cloud_pointsplane->width*cloud_pointsplane->height);

		if(i_mouse<10)
		{
			cloud_pointsplane->points[i_mouse].x = z*(x-px_d)/fx_d;	//x, u-px_d column from the center
    	cloud_pointsplane->points[i_mouse].y = z*(y-py_d)/fy_d;	//y, v-py_d column from the center
    	cloud_pointsplane->points[i_mouse].z = z;								//z,

			i_mouse++;
		}

		//plane segmentation
		if(i_mouse==10)
		{
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

			// Create the segmentation object
  		pcl::SACSegmentation<pcl::PointXYZ> seg;

			// Optional
  		seg.setOptimizeCoefficients (true);

			// Mandatory
  		seg.setModelType (pcl::SACMODEL_PLANE);
  		seg.setMethodType (pcl::SAC_RANSAC);
  		seg.setDistanceThreshold (0.01);

			seg.setInputCloud (cloud_pointsplane);
  		seg.segment (*inliers, *coefficients);

			if (inliers->indices.size () == 0)
  		{
				PCL_ERROR ("Could not estimate a planar model for the given dataset.");
				return;
  		}

			std::cout << std::endl;

			std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                      		<< coefficients->values[1] << " "
                                      		<< coefficients->values[2] << " "
                                      		<< coefficients->values[3] << std::endl;

			std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

			for (size_t i_inl = 0; i_inl < inliers->indices.size (); ++i_inl)
    			std::cerr << inliers->indices[i_inl] << "    " 	<< cloud_pointsplane->points[inliers->indices[i_inl]].x << " "
																											<< cloud_pointsplane->points[inliers->indices[i_inl]].y << " "
                                               				<< cloud_pointsplane->points[inliers->indices[i_inl]].z << std::endl;

      std::cout << std::endl;
			std::cout << "The height of the camera is: "<<coefficients->values[3]/sqrt(coefficients->values[0]*coefficients->values[0]+
																																							 coefficients->values[1]*coefficients->values[1]+
																																							 coefficients->values[2]*coefficients->values[2])<<std::endl;
		}
}

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
  //depth_clean is the true depth in Meter
  cv::Mat depth_clean(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);//True depth
  cv::Mat img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);//For visualization
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
/*
  //Depth to PointCloud
	//pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cloud->width = depth_clean.cols;
	cloud->height = depth_clean.rows;
	cloud->resize(cloud->width*cloud->height);

	for(size_t v=0,i=0; v< depth_clean.rows; v++) //2-D indexing
		for(size_t u=0; u< depth_clean.cols; u++,i++)
			{
				//float z = 1.0f / (depth_clean.at<float>(v,u)*dc1+dc2); //Convert disparity units to true depth
				float z = depth_clean.at<float>(v,u);

				cloud->points[i].x = z*(u-px_d)/fx_d; //u-px_d column from the center
				cloud->points[i].y = z*(v-py_d)/fy_d; //v-py_d column from the center
				cloud->points[i].z = z;

			}
*/
			//Mouse left click
			depthimagemouse=depth_clean;
			cv::namedWindow("depth Image");
		  cv::setMouseCallback( "depth Image", onMouse, 0 );

			//Visualization
      cv::imshow("depth Image", img);
			cvWaitKey(3);
      //cv::imwrite("/depthimages/image8U.png",img);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "heightCamera");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);

	image_transport::Subscriber subkidepth = it.subscribe("/camera/depth/image", 1, imageCallbackdepth);

	ROS_INFO("Estimate the height of the camera");


	ros::spin();
}
