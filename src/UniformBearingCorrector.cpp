#include <UniformBearingCorrector.h>

void UnoformBearingCorrector::getRectifiedSonar(const cv::Mat& img_sonar, const std::vector<int16_t>& bearings, const bool& remap, cv::Mat& img_rect){
    cv::Mat img_polar;
    std::vector<int16_t> bearing_map(bearings);
    int fov = this->azimuth;
    std::for_each(bearing_map.begin(), bearing_map.end(), [fov](int16_t& number) {number += fov*100/2;});    
    
    std::vector<double> linear_map = linspace(1, azimuth*100, img_sonar.cols);
    img_polar = applyAlongAxis(img_sonar, 0, linear_map, bearing_map);
    
    if (remap)
    {
        this->computeRemapMatrices(img_polar, this->map_x, this->map_y, this->angleRes, this->radialRes, fov, this->maxRange);
    }
    cv::remap(img_polar, img_rect, this->map_x, this->map_y, cv::INTER_LINEAR);

    // img_polar.copyTo(img_rect);
    // cv::warpPolar(img_polar, img_rect, img_polar.size(), cv::Point2i(img_sonar.rows/2, 0), img_sonar.cols, cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
}



void UnoformBearingCorrector::computeRemapMatrices(const cv::Mat& polarImg, cv::Mat& map_x, cv::Mat& map_y, const double& angleRes, 
                                                 const double& radialRes, const double& fov, const double& maxRange) {

    // spdlog::info("angleRes: \t{}" , angleRes);
    // spdlog::info("radialRes: \t{}" , radialRes);
    // spdlog::info("fov: \t{}" , fov);
    // spdlog::info("fov: \t{}" , angleRes * polarImg.cols);

    int cartesianWidthPx = 2 * static_cast<int>(polarImg.rows * std::sin((fov * CV_PI / 180.0) / 2));
    int halfSize = cartesianWidthPx / 2;
    map_x.create(polarImg.rows, cartesianWidthPx, CV_32FC1);
    map_y.create(polarImg.rows, cartesianWidthPx, CV_32FC1);

    for (int cv_x = 0; cv_x < cartesianWidthPx; cv_x++) {
        for (int cv_y = 0; cv_y < polarImg.rows; cv_y++) {
            double dx = cv_y;
            double dy = cv_x - halfSize;  // Pixel 0 is far away from actual 0, so it is -halfSize

            double r = std::sqrt(dx*dx + dy*dy);  // radius in [Px]
            double theta = (std::atan2(dy, dx) * 57.27 / angleRes) + (polarImg.cols/2);       // angle in [Px]

            map_x.at<float>(cv_y, cv_x) = theta;
            map_y.at<float>(cv_y, cv_x) = r;

            // if(cv_x == halfSize && cv_y == 256) spdlog::info("At cv: Y{} X{}, Cartesian: X{}, Y{}, Polar: R{}, T{}", cv_y, cv_x, dx, dy, r, theta);
        }
    }

}