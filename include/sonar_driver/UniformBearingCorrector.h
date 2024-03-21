#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"

class UniformBearingCorrector {
public:
    void computeRemapMatrices(const cv::Mat& polarImg, cv::Mat& map_x, cv::Mat& map_y, const double& angleRes, 
                         const double& radialRes, const double& fov, const double& maxRange);

    void getRectifiedSonar(const cv::Mat& img_sonar, const std::vector<int16_t>& bearings, const bool& remap, cv::Mat& img_rect);
};

