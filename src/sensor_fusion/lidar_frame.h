//
// Created by zhanghm on 18-1-15.
//

#ifndef DEPTH_CLUSTERING_LIDARFRAME_H
#define DEPTH_CLUSTERING_LIDARFRAME_H
#include "sensor_fusion/calib_params.h"
#include "utils/cloud.h"


#include <algorithm>
#include <limits>
#include <unordered_map>
#include <vector>

namespace sensor_fusion{

class LidarFrame {
public:
  //useful using
  using Ptr = shared_ptr<LidarFrame>;
  using ConstPtr = shared_ptr<const LidarFrame>;

  //constructor
  explicit LidarFrame(const Cloud::Ptr& cloud,const cv::Mat& camera_image):
          _cloud{cloud},_camera_image{camera_image}{}

  //@brief project clusters to image
  void OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds,
                           int client_id=0);


  /**
   * @brief   project 3D points to camera image.
   *
   */
  void Project2Image();

  //project a single 3D point to image point coordinate
   inline cv::Point point3DTo2D(const depth_clustering::RichPoint& point) const{
    cv::Mat veloPoint3D(4, 1, CV_32F, cv::Scalar(1.0));
    cv::Mat imgPoint2D(3, 1, CV_32F);

    veloPoint3D.at<float>(0, 0) = point.x();
    veloPoint3D.at<float>(1, 0) = point.y();
    veloPoint3D.at<float>(2, 0) = point.z();

    //Important!! eliminate the point behind camera
    //otherwise, it would project redundant points
    if (point.x() < 0)
      return cv::Point(0,0);

    imgPoint2D = _calib_params->ProjectionMatrix() * veloPoint3D;

    if (imgPoint2D.at<float>(2, 0) == 0) {
      fprintf(stderr, "the calculated 2D image points are wrong!\n");
      exit(0);
    }
    //scale transform
    imgPoint2D.at<float>(0, 0) /= imgPoint2D.at<float>(2, 0); //row number
    imgPoint2D.at<float>(1, 0) /= imgPoint2D.at<float>(2, 0); //col number

    int colTmp = int(imgPoint2D.at<float>(0, 0) + 0.5);
    int rowTmp = int(imgPoint2D.at<float>(1, 0) + 0.5);

    if (colTmp < 0 || colTmp > _camera_image.cols || rowTmp < 0 || rowTmp > _camera_image.rows) {
      return cv::Point(0,0);
    }
    cv::Point res(colTmp,rowTmp);

    return res;

  }

  void InitCalibrationParams(const CalibarationParams& params);

  //functions for get class members
  const cv::Mat& camera_image() const{return this->_camera_image;}//return camera_image


  CalibarationParams::Ptr _calib_params;
  depth_clustering::Cloud::Ptr _cloud = nullptr;
  cv::Mat _camera_image;

};


}// namespace sensor_fusion



#endif //DEPTH_CLUSTERING_LIDARFRAME_H
