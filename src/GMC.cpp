#include "GMC.hpp"

namespace n9m::algo {

GMC::GMC(const Config& cfg) : m_cfg(cfg) {}

cv::Mat GMC::computeTransform(const cv::Mat& prevGray, const cv::Mat& currGray, const cv::Rect2d& droneRoi) {
    // Respect the 'enabled' flag from JSON
    if (!m_cfg.gmc.enabled || prevGray.empty() || currGray.empty()) {
        return cv::Mat::eye(2, 3, CV_64F);
    }

    // Mask out the drone to ensure we only track background motion
    // TODO: Use the homography flag if needed
    
    cv::Mat mask = cv::Mat::ones(prevGray.size(), CV_8U) * 255;
    cv::Rect maskRect = static_cast<cv::Rect>(droneRoi);
    // Buffer the mask slightly to avoid drone edge artifacts
    maskRect.x -= 20; maskRect.y -= 20; maskRect.width += 40; maskRect.height += 40;
    maskRect &= cv::Rect(0, 0, prevGray.cols, prevGray.rows);
    if (maskRect.area() > 0) mask(maskRect) = 0;

    // Feature Detection (Good Features To Track)
    // Uses 'maxFeatures' from config to control the number of points for better performance
    std::vector<cv::Point2f> prevPts;
    cv::goodFeaturesToTrack(prevGray, prevPts, 
                            m_cfg.gmc.maxFeatures, 
                            0.01, // Quality level
                            8.0,  // Min distance
                            mask);

    if (prevPts.size() < 20) return cv::Mat::eye(2, 3, CV_64F);

    // KLT Optical Flow
    std::vector<cv::Point2f> currPts;
    std::vector<uchar> status;
    std::vector<float> err;
    
    // Pyramid level 4 helps capture rapid pans
    cv::calcOpticalFlowPyrLK(prevGray, currGray, prevPts, currPts, status, err, cv::Size(31, 31), 4);

    // Filter for valid movement
    std::vector<cv::Point2f> validPrev, validCurr;
    for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) {
            validPrev.push_back(prevPts[i]);
            validCurr.push_back(currPts[i]);
        }
    }

    if (validPrev.size() < 15) return cv::Mat::eye(2, 3, CV_64F);

    // Estimate Partial Affine (Gold Standard for Tilt/Translation)
    cv::Mat affine = cv::estimateAffinePartial2D(validPrev, validCurr, cv::noArray(), cv::RANSAC, 3.0);

    return affine.empty() ? cv::Mat::eye(2, 3, CV_64F) : affine;
}

} // namespace n9m::algo