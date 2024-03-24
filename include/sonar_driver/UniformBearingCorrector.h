#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"

class UniformBearingCorrector {
public:
    UniformBearingCorrector(double angularResolution, double rangeResolution, double minRange, double maxRange) : 
        angularResolution_(angularResolution), rangeResolution_(rangeResolution), minRange_(minRange), maxRange_(maxRange){}
    void computeRemapMatrices(const cv::Mat& polarImg, cv::Mat& map_x, cv::Mat& map_y, const double& angleRes, 
                         const double& rangeRes, const double& fov, const double& maxRange);

    void rectifyImage(const cv::Mat& img_sonar, cv::Mat& img_uniform, const std::vector<int16_t>& bearings, cv::Mat& img_rect);

    static std::vector<double> linspace(const int& start_in, const int& end_in, const int& num_in);
    static cv::Mat interp(const std::vector<int16_t>& xp, const cv::Mat& yp, const std::vector<double>& x);

    static cv::Mat applyAlongAxis(const cv::Mat& inputMatrix, const int& axis, const std::vector<double>& linearMap, const std::vector<int16_t>& bearingMap);
private:
    double angularResolution_ = 0;
    double rangeResolution_ = 0;
    double minRange_ = 0.1;
    double maxRange_ = 10.0;
};

