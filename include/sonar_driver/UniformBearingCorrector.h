#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"


struct UniformBearingCorrectorConfig{
    UniformBearingCorrectorConfig(int rows, int cols,
                                  double minRange, double maxRange, 
                                  double angularResolution) : 
                                  rows(rows), cols(cols), minRange(minRange), maxRange(maxRange), angularResolution(angularResolution){
        this->fov = cols * angularResolution;
        this->rangeResolution = (maxRange - minRange) / (double)rows;
        spdlog::info("UniformBearingCorrectorConfig: Size {}/{}, Range: {}/{}, Resolution: {}/{}, FOV: {}", rows, cols, minRange, maxRange, rangeResolution, angularResolution, fov);
    }

    void setBearings(const std::vector<int16_t>& bearings){
        bearingMap = std::vector<int16_t>(bearings);
        double tempFov = (bearings.back() - bearings.front()); 
        std::for_each(bearingMap.begin(), bearingMap.end(), [tempFov](int16_t& number) {number += tempFov/2;});   
    }

    int rows = 1;
    int cols = 1;
    double angularResolution = 0.0;
    double rangeResolution = 0.0;
    double fov = 0.0;
    double minRange = 0.0;
    double maxRange = 0.0;
    std::vector<int16_t> bearingMap; 

    bool operator==(const UniformBearingCorrectorConfig& config) const{
        return config.rows == this->rows && config.cols == this->cols &&
               config.angularResolution == this->angularResolution && config.rangeResolution == this->rangeResolution &&
               config.minRange == this->minRange && config.maxRange == this->maxRange; 
    }
};



class UniformBearingCorrector {
public:
    UniformBearingCorrector(const UniformBearingCorrectorConfig& config);
    void computeRemapMatrices(cv::Mat& mapX, cv::Mat& mapY);
    void computeUniformRemapMatrices(cv::Mat& mapX, cv::Mat& mapY);
    

    void rectifyImage(const cv::Mat& img_sonar, cv::Mat& img_uniform, cv::Mat& img_rect);

    static std::vector<double> linspace(const int& start_in, const int& end_in, const int& num_in);
    static cv::Mat interp(const std::vector<int16_t>& xp, const cv::Mat& yp, const std::vector<double>& x);

    static cv::Mat applyAlongAxis(const cv::Mat& inputMatrix, const int& axis, const std::vector<double>& linearMap, const std::vector<int16_t>& bearingMap);

    bool hasSameConfig(const UniformBearingCorrectorConfig& config) const{
        return config == config_;
    }
    
private:
    UniformBearingCorrectorConfig config_;
    cv::Mat mapX_;
    cv::Mat mapY_;
    cv::Mat uniformMapX_;
    cv::Mat uniformMapY_;
};

