//
// Created by zhanghm on 18-1-15.
//

#ifndef DEPTH_CLUSTERING_CALIB_PARAMS_H
#define DEPTH_CLUSTERING_CALIB_PARAMS_H

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <vector>

#include "utils/useful_typedefs.h" //for use shared_ptr and make_shared
using namespace depth_clustering;

namespace sensor_fusion {

/**
 * @brief      Class for storing kitti camera and velodyne lidar projection parameters.
 */
class CalibarationParams {
public:
  using Ptr = shared_ptr<CalibarationParams>;
  using ConstPtr = const shared_ptr<const CalibarationParams>;

  //Constructors
  CalibarationParams()
          : _cameraMatrix(cv::Mat(3, 4, CV_32F, cv::Scalar::all(0))),
            _velo2camera(cv::Mat(4, 4, CV_32F, cv::Scalar::all(0))) {
    _velo2camera.at<float>(3, 3) = 1.0;
  }

  ~CalibarationParams() {}

  inline cv::Mat CameraMatrix() const { return _cameraMatrix; }

  inline cv::Mat ProjectionMatrix() const { return _projectionMatrix; }

  inline cv::Mat Velo2CameraCoord() const { return _velo2camera; }

  /**
   * @brief      From file diretory to read calibration parameters
   *
   * @return     A pointer to parameters
   */
  static std::unique_ptr<CalibarationParams> FromCalibFile(const std::string &calibFile);

  cv::Mat _projectionMatrix; //project 3D point in velodyne lidar to camera image
  cv::Mat _cameraMatrix;//project 3D point under camera0 coordinate to other camera image , to image of camera2 by default
  cv::Mat _velo2camera;//Tr,velodyne lidar point cloud to camera0 world coordinate

};

}

#endif //DEPTH_CLUSTERING_CALIB_PARAMS_H
