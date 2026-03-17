#pragma once

#include "Config.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

namespace n9m::algo {

class GMC {
public:
    
    GMC(const Config& cfg);

    /**
     * @brief KLT-based Global Motion Compensation.
     * High density (1000 pts) for pocket-texture tracking.
     */
    cv::Mat computeTransform(const cv::Mat& prevGray, const cv::Mat& currGray, const cv::Rect2d& droneRoi);

private:
    const Config& m_cfg; 
};

} // namespace n9m::algo