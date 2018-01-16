//
// Created by zhanghm on 18-1-15.
//

#ifndef DEPTH_CLUSTERING_LIDARFRAME_H
#define DEPTH_CLUSTERING_LIDARFRAME_H
#include "sensor_fusion/calib_params.h"
#include "utils/cloud.h"
namespace sensor_fusion{

class LidarFrame {
public:
  //constructor
  explicit LidarFrame(const Cloud::Ptr& cloud,const cv::Mat& camera_image):
          _cloud{cloud},_camera_image{camera_image}{}


  /**
   * @brief   project 3D point to camera image.
   *
   */
  void Project2Image();

  void InitCalibrationParams(const CalibarationParams& params);

  //functions for get class members
  const cv::Mat& camera_image() const{return this->_camera_image;}//return camera_image


  CalibarationParams::Ptr _calib_params;
  depth_clustering::Cloud::Ptr _cloud = nullptr;
  cv::Mat _camera_image;

};


}// namespace sensor_fusion



#endif //DEPTH_CLUSTERING_LIDARFRAME_H
