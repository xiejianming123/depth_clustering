//
// Created by zhanghm on 18-1-15.
//

#include "./lidar_frame.h"
#include "qt/utils/utils.h"
#include "qt/drawables/drawable_cloud.h"
namespace sensor_fusion{

void LidarFrame::InitCalibrationParams(const CalibarationParams& params)
{
  _calib_params = CalibarationParams::Ptr(new CalibarationParams(params));

}

void LidarFrame::OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds,
                                          int client_id){
  for(const auto& kv:clouds){
    const auto& cluster = kv.second; //one of cloud cluster

    //visualize the 3D clusters
    Cloud::Ptr segments = boost::make_shared<Cloud>(cluster);
    Eigen::Vector3f color = randomColor();
    _viewer->AddDrawable(DrawableCloud::FromCloud(segments,color));

    color = 255*color;
    for(const auto& point:cluster.points()){
      cv::Point image_point = point3DTo2D(point);
      cv::circle(_camera_image,image_point,1,cv::Scalar(color.z(),color.y(),color.x()));

    }
  }

  _viewer->update();

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