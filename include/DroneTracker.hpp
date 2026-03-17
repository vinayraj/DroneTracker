#pragma once

#include "Config.hpp"
#include "GMC.hpp"

#include <opencv2/opencv.hpp>

#include <memory>
#include <vector>

namespace n9m {

/**
 * @brief High-performance drone tracker utilizing Global Motion Compensation (GMC),
 * normalized cross-correlation with adaptive spatial anchoring, and physics-based
 * state prediction.
 * * Optimized for drone interception scenarios where rapid camera pans 
 * and low-contrast backgrounds (e.g., green-on-green) are common.
 */
class DroneTracker {
public:
    /**
     * @brief Constructor
     * @param cfg Unified configuration containing dynamics and detection params.
     */
    explicit DroneTracker(const Config& cfg);

    /**
     * @brief Processes the current frame and returns the updated drone ROI.
     * @details Internal pipeline:
     * 1. Downsampling for performance scaling.
     * 2. Background alignment (GMC).
     * 3. Search window prediction based on velocity.
     * 4. Template matching with dynamic spatial penalty.
     * 5. Adaptive gain update (Pursuit vs. Stationary).
     * * @param currFrame The latest BGR frame from the camera.
     * @return cv::Rect2d The new bounding box for the drone in original-scale coordinates.
     */
    cv::Rect2d update(const cv::Mat& currFrame);

    double getInternalLatency() const { return m_lastLogicLatencyMs; }

    cv::Mat getDiagnosticCanvas() const { return m_canvas; }  
    
private:
    // --- State & Config ---
    Config m_cfg;
    cv::Rect2d m_currentRoi; // Tracked ROI in the processing (downsampled) space
    bool m_initialized;

    // --- Background Alignment ---
    std::unique_ptr<algo::GMC> m_gmc;
    cv::Mat m_prevGray; // Previous frame in downsampled space

    // --- Appearance & Motion ---
    cv::Mat m_template;    // Visual anchor (drone appearance)
    cv::Point2d m_velocity; // Independent motion vector in processing space
    
    // --- Performance Optimization ---
    cv::Ptr<cv::CLAHE> m_clahe; // Pre-processing for contrast enhancement

    double m_lastLogicLatencyMs = 0.0;

    cv::Mat m_canvas; // Diagnostic overlay for debugging
};

} // namespace n9m