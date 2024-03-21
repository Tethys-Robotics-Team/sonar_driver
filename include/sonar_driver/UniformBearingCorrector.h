#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"

class UniformBearingCorrector {
public:
    computeRemapMatrices(const cv::Mat& polarImg, cv::Mat& map_x, cv::Mat& map_y, const double& angleRes, 
                         const double& radialRes, const double& fov, const double& maxRange)

    getRectifiedSonar(const cv::Mat& img_sonar, const std::vector<int16_t>& bearings, const bool& remap, cv::Mat& img_rect)
};

