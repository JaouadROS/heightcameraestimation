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

// C++
#include <iostream>
#include <string>

// OpenCv
#include <opencv/cv.h>
#include <opencv/highgui.h>

// PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//Camera_Height header
#include "height_camera_estimation/Camera_Height.h"

CameraHeight::CameraHeight(const std::string &winname, const cv::Mat depth_clean)
: m_winname_(winname), depth_image(depth_clean)
{
  fx_ = 5.9421434211923247e+02,
  fy_ = -5.9104053696870778e+02,
  px_ = 320,
  py_ = 240;

  //i_mouse=0;

  //cloud_pointsplane.reset(new pcl::PointCloud<pcl::PointXYZ>);

  cv::namedWindow(m_winname_);
  cv::setMouseCallback(m_winname_, &CameraHeight::onMouse, this);
}

CameraHeight::~CameraHeight()
{
  cv::setMouseCallback(m_winname_, NULL, 0);
}

void CameraHeight::onMouse( int event, int x, int y, int flags, void* param )
{
  CameraHeight *anInstance = static_cast<CameraHeight *>(param);

  if( event != CV_EVENT_LBUTTONDOWN )
  return;

  // window mouse coordinates
  cv::Point pt = cv::Point(x,y);

  float z=anInstance->depth_image.at<float>(y,x);
  static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pointsplane (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // plane segmentation of a set of 10 points
  cloud_pointsplane->width = 10;
  cloud_pointsplane->height = 1;
  cloud_pointsplane->points.resize(cloud_pointsplane->width*cloud_pointsplane->height);

  static size_t i_mouse=0;

  // Store the data
  if(i_mouse<10)
  {
    cloud_pointsplane->points[i_mouse].x = z*(x-anInstance->px_)/anInstance->fx_;	//x, u-px_ column from the center
    cloud_pointsplane->points[i_mouse].y = z*(y-anInstance->py_)/anInstance->fy_;	//y, v-py_ column from the center
    cloud_pointsplane->points[i_mouse].z = z;								//z,
    i_mouse++;
  }

  if (i_mouse==10){
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
    std::cerr << inliers->indices[i_inl] << "    "
              << cloud_pointsplane->points[inliers->indices[i_inl]].x << " "
              << cloud_pointsplane->points[inliers->indices[i_inl]].y << " "
              << cloud_pointsplane->points[inliers->indices[i_inl]].z << std::endl;

    std::cout << std::endl;

    std::cout << "The height of the camera is: "
              <<coefficients->values[3]/sqrt( coefficients->values[0]*coefficients->values[0]+
                                              coefficients->values[1]*coefficients->values[1]+
                                              coefficients->values[2]*coefficients->values[2])<<" (m)"<<std::endl;
    }
  }
