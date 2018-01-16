// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include "./utils.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <utils/folder_reader.h>
#include <utils/velodyne_utils.h>
#include <projections/projection_params.h>

#include <QDebug>
using depth_clustering::Cloud;
using depth_clustering::ProjectionParams;
using depth_clustering::MatFromDepthPng;
using depth_clustering::ReadKittiCloudTxt;
using depth_clustering::ReadKittiCloud;

QString appendPaths(const QString &path1, const QString &path2) {
  return QDir::cleanPath(path1 + QDir::separator() + path2);
}

QImage MatToQImage(const cv::Mat &image) {
  auto qimage = QImage(image.cols, image.rows, QImage::Format_RGB888);
  if (image.type() == CV_32F) {
    for (int r = 0; r < image.rows; ++r) {
      for (int c = 0; c < image.cols; ++c) {
        if (image.at<float>(r, c) == 666) {
          auto color = qRgb(0, 200, 0);
          qimage.setPixel(c, r, color);
          continue;
        }
        const float &val = image.at<float>(r, c) * 10;
        auto color = qRgb(val, val, val);
        qimage.setPixel(c, r, color);
      }
    }
  } else {
    for (int r = 0; r < image.rows; ++r) {
      for (int c = 0; c < image.cols; ++c) {
        auto val = image.at<cv::Vec3b>(r, c);
        auto color = qRgb(val[0], val[1], val[2]);
        qimage.setPixel(c, r, color);
      }
    }
  }
  return qimage;
}

Cloud::Ptr CloudFromFile(const std::string &file_name,
                         const ProjectionParams &proj_params) {
  QFileInfo fi(QString::fromStdString(file_name));
  QString name = fi.fileName();
  Cloud::Ptr cloud = nullptr;
  if (name.endsWith(".pcd")) {
    pcl::PointCloud<pcl::PointXYZL> pcl_cloud;
    pcl::io::loadPCDFile(file_name, pcl_cloud);
    cloud = Cloud::FromPcl<pcl::PointXYZL>(pcl_cloud);
    cloud->InitProjection(proj_params);
  } else if (name.endsWith(".png") || name.endsWith(".exr")) {
    cloud = Cloud::FromImage(MatFromDepthPng(file_name), proj_params);
  } else if (name.endsWith(".txt")) {
    cloud = ReadKittiCloudTxt(file_name);
    cloud->InitProjection(proj_params);
  } else if (name.endsWith(".bin")) {
    cloud = ReadKittiCloud(file_name);
    cloud->InitProjection(proj_params);
  }
  // if (cloud) {
  //   // if the cloud was actually set, then set the name in the gui
  //   ui->lbl_cloud_name->setText(name);
  // }
  return cloud;
}

//@folder_dir the image folder's parent folder path
bool ReadCameraImage(const std::string& file_name,cv::Mat& imgRead,std::string& parent_dir_path){
  QFileInfo fi(QString::fromStdString(file_name));
  QString name = fi.fileName();
  QString basename = fi.baseName();
  QString path = fi.path();
  if(name.endsWith(".bin"))
  {
    std::string temp = path.toStdString();
    std::string prefix = temp.substr(0,temp.find_last_of('/')+1);
    std::string imagePath = prefix+"image_2/"+basename.toStdString()+".png";
    std::cout<<imagePath<<std::endl;
    imgRead = cv::imread(imagePath, CV_LOAD_IMAGE_COLOR);//CV_8UC3 type

    parent_dir_path = prefix;
    return true;
  }
  printf("[INFO]: without corresponding camera image, please check!");
  return false;
}