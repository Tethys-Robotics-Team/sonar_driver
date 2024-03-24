
#include <sonar_driver/UniformBearingCorrector.h>

std::vector<double> UniformBearingCorrector::linspace(const int& start_in, const int& end_in, const int& num_in)
{
    std::vector<double> linspaced;
    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in);
    double num = static_cast<double>(num_in);

    if (num == 0) { return linspaced; }
    if (num == 1) {
      linspaced.push_back(start);
      return linspaced;
    }

    double delta = (end - start) / (num - 1);
  
    for(int i=0; i < num-1; ++i)    {
      linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // I want to ensure that start and end
                              // are exactly the same as the input
    return linspaced;
}

/// @brief Interpolates the values of a Matrix with known column spacing xp to another column spacing x
/// @param xp 
/// @param yp 
/// @param x 
/// @return 
cv::Mat UniformBearingCorrector::interp(const std::vector<int16_t>& xp, const cv::Mat& yp, const std::vector<double>& x){
    int N_x = x.size();
    int N_xp = xp.size();
	cv::Mat y(yp.size(), CV_8UC1);

	for(int i = 0; i<N_x; i++) y.at<u_int8_t>(0, i) = 0;

	int ip = 0;
	int ip_next = 1;
	int i = 0;

	while(i < N_x){
		double m = static_cast<double>(yp.at<u_int8_t>(0, ip_next)-yp.at<u_int8_t>(0, ip))/(static_cast<double>(xp[ip_next]-xp[ip])/100.0);
		double q = static_cast<double>(yp.at<u_int8_t>(0, ip)) - m * static_cast<double>(xp[ip])/100.0;
		
        while(x[i] < xp[ip_next]){
			if(x[i] >= xp[ip]){
                y.at<u_int8_t>(0, i) = static_cast<u_int8_t>(std::round(m*x[i]/100.0+q));

			}
			i +=1;
			if (i >= N_x) {printf("break\n"); break;}
		}

    	ip +=1;
		ip_next +=1;
		if(ip_next == N_xp){
			while(i < N_x){
				y.at<u_int8_t>(0, i) = 0;
				i++;
			}
			break;
		}
	}

	return y;
}

/// @brief 
/// @param inputMatrix image
/// @param axis image
/// @param linearMap image
/// @param bearingMap image
cv::Mat UniformBearingCorrector::applyAlongAxis(const cv::Mat& inputMatrix, const int& axis, const std::vector<double>& linearMap, const std::vector<int16_t>& bearingMap) {
    cv::Mat outputMatrix = cv::Mat(inputMatrix.rows, inputMatrix.cols, inputMatrix.type());
    
    // Apply along rows
    if (axis == 0) {  
        for (int i = 0; i < inputMatrix.rows; i++) {
            cv::Mat inputRow = inputMatrix.row(i);
            cv::Mat outputRow = interp(bearingMap, inputRow, linearMap);
            outputRow.copyTo(outputMatrix.row(i));
        }
    }
    // Apply Along Columns
    else if (axis == 1) { 
        for (int i = 0; i < inputMatrix.cols; i++) {
            cv::Mat inputCol = inputMatrix.col(i);
            cv::Mat outputCol = interp(bearingMap, inputCol, linearMap);
            outputCol.copyTo(outputMatrix.col(i));
        }
    } else {
        spdlog::error("UniformBearingCorrector: Wrong axis: {}", axis);
    }

    return outputMatrix;
}

void UniformBearingCorrector::rectifyImage(const cv::Mat& img_sonar, cv::Mat& img_uniform, const std::vector<int16_t>& bearings, cv::Mat& img_rect){
    std::vector<int16_t> bearingMap(bearings);
    int fov = (bearings.back() - bearings.front()) * 100 / 2; 
    std::for_each(bearingMap.begin(), bearingMap.end(), [fov](int16_t& number) {number += fov*100/2;});    
    
    std::vector<double> linearMap = linspace(1, fov, img_sonar.cols);
    img_uniform = applyAlongAxis(img_sonar, 0, linearMap, bearingMap);
    
    if (true){
        cv::Mat map_x, map_y;
        this->computeRemapMatrices(img_uniform, map_x, map_y, this->angularResolution_, this->rangeResolution_, fov, this->maxRange_);
        cv::remap(img_uniform, img_rect, map_x, map_y, cv::INTER_LINEAR);
    }
}



void UniformBearingCorrector::computeRemapMatrices(const cv::Mat& polarImg, cv::Mat& map_x, cv::Mat& map_y, const double& angleRes, 
                                                 const double& rangeRes, const double& fov, const double& maxRange) {

    // spdlog::info("angleRes: \t{}" , angleRes);
    // spdlog::info("rangeRes: \t{}" , rangeRes);
    // spdlog::info("fov: \t{}" , fov);
    // spdlog::info("fov: \t{}" , angleRes * polarImg.cols);

    int cartesianWidthPx = 2 * static_cast<int>(polarImg.rows * std::sin((fov * CV_PI / 180.0) / 2));
    int halfSize = cartesianWidthPx / 2;
    map_x.create(polarImg.rows, cartesianWidthPx, CV_32FC1);
    map_y.create(polarImg.rows, cartesianWidthPx, CV_32FC1);

    for (int cv_x = 0; cv_x < cartesianWidthPx; cv_x++) {
        for (int cv_y = 0; cv_y < polarImg.rows; cv_y++) {
            double dx = cv_y;
            double dy = cv_x - halfSize;  // Pixel 0 is left to the angle 0, so it is -halfSize

            double r = std::sqrt(dx*dx + dy*dy);  // radius in [Px]
            double theta = (std::atan2(dy, dx) * 57.27 / angleRes) + (polarImg.cols/2);       // angle in [Px]

            map_x.at<float>(cv_y, cv_x) = theta;
            map_y.at<float>(cv_y, cv_x) = r;

            // if(cv_x == halfSize && cv_y == 256) spdlog::info("At cv: Y{} X{}, Cartesian: X{}, Y{}, Polar: R{}, T{}", cv_y, cv_x, dx, dy, r, theta);
        }
    }

}