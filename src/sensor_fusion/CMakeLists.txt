add_library(lidar_frame SHARED lidar_frame.cpp calib_params.cpp)
target_link_libraries(lidar_frame
                      ${OpenCV_LIBS})