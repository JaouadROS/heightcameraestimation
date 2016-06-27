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

// C++ string
#include <string>

//OpenCv
#include <opencv/cv.h>

class CameraHeight
{
  float fx_, fy_, px_, py_;
  cv::Mat depth_image;
  std::string m_winname_;

public:
  CameraHeight(const std::string &winname, const cv::Mat depth_clean);
  ~CameraHeight();
  static void onMouse( int evt, int x, int y, int flags, void* param );
};
