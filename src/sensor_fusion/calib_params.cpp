//
// Created by zhanghm on 18-1-15.
//

#include "sensor_fusion/calib_params.h"
#include <sstream>
#include <iostream>
#include <fstream>

#include "utils/mem_utils.h"
std::unique_ptr<CalibarationParams> CalibarationParams::FromCalibFile(const std::string& calibFile){
  auto params = CalibarationParams();
  std::ifstream inputFile;
  inputFile.open(calibFile.c_str(),std::ios_base::in);
  if(!inputFile.is_open())
  {
    fprintf(stderr, "cannot open the calibration file: %s!\n", calibFile.c_str());
    return nullptr;
  }

  while( !inputFile.eof() )
  {
    std::string line_tmp;
    std::getline(inputFile,line_tmp); //read one line
    std::istringstream inputString(line_tmp);//define a string stream
    std::string tag;
    inputString>>tag;

    //just read the color camera of P2 for this function
    if ( tag == "P2:")
    {
      inputString>> params._cameraMatrix.at<float>(0,0) >> params._cameraMatrix.at<float>(0,1) >> params._cameraMatrix.at<float>(0,2) >> params._cameraMatrix.at<float>(0,3)
                 >> params._cameraMatrix.at<float>(1,0) >> params._cameraMatrix.at<float>(1,1) >> params._cameraMatrix.at<float>(1,2) >> params._cameraMatrix.at<float>(1,3)
                 >> params._cameraMatrix.at<float>(2,0) >> params._cameraMatrix.at<float>(2,1) >> params._cameraMatrix.at<float>(2,2) >> params._cameraMatrix.at<float>(2,3);
    }

    if(tag == "Tr:")
    {
      inputString >> params._velo2camera.at<float>(0,0) >> params._velo2camera.at<float>(0,1) >> params._velo2camera.at<float>(0,2) >> params._velo2camera.at<float>(0,3)
                  >> params._velo2camera.at<float>(1,0) >> params._velo2camera.at<float>(1,1) >> params._velo2camera.at<float>(1,2) >> params._velo2camera.at<float>(1,3)
                  >> params._velo2camera.at<float>(2,0) >> params._velo2camera.at<float>(2,1) >> params._velo2camera.at<float>(2,2) >> params._velo2camera.at<float>(2,3);
    }

  }
  inputFile.close();

  //projectionMatrix = 3X4;
  params._projectionMatrix = params._cameraMatrix*params._velo2camera;//project 3D point in lidar to camera image coordinate

  return mem_utils::make_unique<CalibarationParams>(params);

}
