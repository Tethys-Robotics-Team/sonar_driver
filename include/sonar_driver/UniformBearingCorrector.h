#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"


UniformBearingCorrectorConfig{
    int rows;
    int cols;
    double angularResolution;
    double rangeResolution;
    double minRange;
    double maxRange;
}



class UniformBearingCorrector {
public:
    UniformBearingCorrector(UniformBearingCorrectorConfig config);
    void computeRemapMatrices(const UniformBearingCorrectorConfig& config, cv::Mat& mapX, cv::Mat& mapY);

    void rectifyImage(const cv::Mat& img_sonar, cv::Mat& img_uniform, const std::vector<int16_t>& bearings, cv::Mat& img_rect);

    static std::vector<double> linspace(const int& start_in, const int& end_in, const int& num_in);
    static cv::Mat interp(const std::vector<int16_t>& xp, const cv::Mat& yp, const std::vector<double>& x);

    static cv::Mat applyAlongAxis(const cv::Mat& inputMatrix, const int& axis, const std::vector<double>& linearMap, const std::vector<int16_t>& bearingMap);
private:
    UniformBearingCorrectorConfig config_;
    cv::Mat mapX_;
    cv::Mat mapY_;
};

