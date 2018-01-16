//
// Created by zhanghm on 18-1-15.
//

#include "lidar_frame.h"

namespace sensor_fusion{

void LidarFrame::InitCalibrationParams(const CalibarationParams& params)
{
  _calib_params = CalibarationParams::Ptr(new CalibarationParams(params));

}

void LidarFrame::Project2Image() {
  if (!_calib_params) {
    fprintf(stderr, "ERROR: failed to initalize calibration parameters.\n");
    return;
  }

  for (const auto &point : _cloud->points()) {
    cv::Mat veloPoint3D(4, 1, CV_32F, cv::Scalar(1.0));
    cv::Mat imgPoint2D(3, 1, CV_32F);

    veloPoint3D.at<float>(0, 0) = point.x();
    veloPoint3D.at<float>(1, 0) = point.y();
    veloPoint3D.at<float>(2, 0) = point.z();

    //Important!! eliminate the point behind camera
    //otherwise, it would project redundant points
    if (point.x() < 0)
      continue;

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
      continue;
    }

    cv::circle(_camera_image, cv::Point(colTmp, rowTmp), 1, cv::Scalar(0, 0, 255));


  }
}


}// namespace sensor_fusion