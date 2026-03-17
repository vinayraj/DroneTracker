#include "DroneTracker.hpp"
#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>
#include <chrono>

namespace n9m {

DroneTracker::DroneTracker(const Config& cfg) 
    : m_cfg(cfg), m_initialized(false), m_velocity(0.0, 0.0) {
    m_gmc = std::make_unique<algo::GMC>(m_cfg);
    m_clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
}

cv::Rect2d DroneTracker::update(const cv::Mat& currFrame) {
    if (currFrame.empty()) return m_currentRoi;

    // --- START CORE PERCEPTION TIMER ---
    auto startLogic = std::chrono::steady_clock::now();

    // Downsample: Normalize for performance scaling
    double scale = 1.0;
    cv::Mat processingFrame;
    if (currFrame.cols > m_cfg.tracker.processingWidth) {
        scale = m_cfg.tracker.processingWidth / currFrame.cols;
        cv::resize(currFrame, processingFrame, cv::Size(), scale, scale, cv::INTER_LINEAR);
    } else {
        processingFrame = currFrame;
    }

    cv::Mat currGray;
    cv::cvtColor(processingFrame, currGray, cv::COLOR_BGR2GRAY);

    if (!m_initialized) {
        // Sync internal state to downsampled coordinates
        m_currentRoi = cv::Rect2d(m_cfg.initialRoi.x * scale, m_cfg.initialRoi.y * scale,
                                  m_cfg.initialRoi.width * scale, m_cfg.initialRoi.height * scale);
        
        cv::Rect safeRect = static_cast<cv::Rect>(m_currentRoi) & cv::Rect(0, 0, currGray.cols, currGray.rows);
        if (safeRect.area() > 0) m_template = currGray(safeRect).clone();
        
        m_prevGray = currGray.clone();
        m_initialized = true;
        m_lastLogicLatencyMs = 0.0;
        return m_cfg.initialRoi;
    }

    // GMC: Predict Background Shift
    cv::Mat affine = m_gmc->computeTransform(m_prevGray, currGray, m_currentRoi);
    double tx = affine.at<double>(0, 2);
    double ty = affine.at<double>(1, 2);
    double panMag = std::sqrt(tx * tx + ty * ty);

    // Calculate Motion Energy
    cv::Mat warpedPrev, motionEnergy;
    cv::warpAffine(m_prevGray, warpedPrev, affine, currGray.size());
    cv::absdiff(currGray, warpedPrev, motionEnergy);
    cv::threshold(motionEnergy, motionEnergy, 25, 255, cv::THRESH_BINARY);

    // Adaptive Search Window & Template Matching
    int currentPad = m_cfg.tracker.searchPaddingBase;
    float currentSigma = static_cast<float>(m_cfg.tracker.spatialPenaltySigma);
    double currentThreshold = m_cfg.tracker.matchThresholdNormal;

    if (panMag > m_cfg.tracker.panThreshold) { 
        currentPad = static_cast<int>(panMag * m_cfg.tracker.searchPaddingScale);
        currentSigma = static_cast<float>(panMag * 4.0);
        currentThreshold = m_cfg.tracker.matchThresholdRapid;
    }

    cv::Rect2d predRoi = m_currentRoi;
    predRoi.x += (tx + m_velocity.x);
    predRoi.y += (ty + m_velocity.y);

    cv::Rect vWin = static_cast<cv::Rect>(predRoi);
    vWin.x -= currentPad; vWin.y -= currentPad; 
    vWin.width += 2 * currentPad; vWin.height += 2 * currentPad;
    vWin &= cv::Rect(0, 0, currGray.cols, currGray.rows);

    cv::Point2d targetCenter(m_currentRoi.x + m_currentRoi.width / 2.0, 
                             m_currentRoi.y + m_currentRoi.height / 2.0);

    bool detected = false;
    double confidence = 0.0;
    cv::Mat res;

    if (vWin.width > m_template.cols && vWin.height > m_template.rows) {
        cv::matchTemplate(currGray(vWin), m_template, res, cv::TM_CCOEFF_NORMED);
        
        // Spatial Penalty Map
        float cX = res.cols / 2.0f;
        float cY = res.rows / 2.0f;
        for (int y = 0; y < res.rows; y++) {
            float* row = res.ptr<float>(y);
            for (int x = 0; x < res.cols; x++) {
                float d2 = static_cast<float>(std::pow(x - cX, 2) + std::pow(y - cY, 2));
                row[x] *= std::exp(-d2 / (2.0f * currentSigma * currentSigma));
            }
        }

        double maxVal; cv::Point maxLoc;
        cv::minMaxLoc(res, nullptr, &maxVal, nullptr, &maxLoc);
        confidence = maxVal;

        if (maxVal > currentThreshold) {
            // Sub-pixel Refinement
            double subX = maxLoc.x;
            double subY = maxLoc.y;
            if (maxLoc.x > 0 && maxLoc.x < res.cols - 1 && maxLoc.y > 0 && maxLoc.y < res.rows - 1) {
                float l = res.at<float>(maxLoc.y, maxLoc.x - 1);
                float r = res.at<float>(maxLoc.y, maxLoc.x + 1);
                float c = res.at<float>(maxLoc.y, maxLoc.x);
                subX += (l - r) / (2.0f * (l + r - 2.0f * c + 1e-5f));

                float t = res.at<float>(maxLoc.y - 1, maxLoc.x);
                float b = res.at<float>(maxLoc.y + 1, maxLoc.x);
                subY += (t - b) / (2.0f * (t + b - 2.0f * c + 1e-5f));
            }

            cv::Point2d obsCenter(vWin.x + subX + m_template.cols / 2.0, 
                                  vWin.y + subY + m_template.rows / 2.0);

            cv::Point2d error = obsCenter - (targetCenter + cv::Point2d(tx, ty));
            double errNorm = cv::norm(error);

            // Dynamic Response Gain
            double alpha = (panMag > m_cfg.tracker.panThreshold || errNorm > 1.5) 
                           ? m_cfg.tracker.alphaPursuit : m_cfg.tracker.alphaStationary;
            
            double vResp = 1.0 - m_cfg.tracker.velocityMemory;
            m_velocity.x = m_velocity.x * m_cfg.tracker.velocityMemory + error.x * vResp;
            m_velocity.y = m_velocity.y * m_cfg.tracker.velocityMemory + error.y * vResp;

            targetCenter.x += tx + error.x * alpha;
            targetCenter.y += ty + error.y * alpha;
            detected = true;
        }
    }

    if (!detected) {
        targetCenter.x += tx + m_velocity.x;
        targetCenter.y += ty + m_velocity.y;
        m_velocity *= m_cfg.tracker.fallbackDecay;
    }

    // Update Internal State
    m_currentRoi.x = targetCenter.x - m_currentRoi.width / 2.0;
    m_currentRoi.y = targetCenter.y - m_currentRoi.height / 2.0;
    m_prevGray = currGray.clone();

    // Output Diagnostic Info and Visualization
    cv::Rect2d finalRoi(m_currentRoi.x / scale, m_currentRoi.y / scale,
                        m_currentRoi.width / scale, m_currentRoi.height / scale);

    
    // --- Stop perception timer ---
    auto endLogic = std::chrono::steady_clock::now();
    m_lastLogicLatencyMs = std::chrono::duration_cast<std::chrono::microseconds>(endLogic - startLogic).count() / 1000.0;
    
    // --- Log Diagnostic Information ---
    spdlog::info("Drone Localized: [x:{:.1f}, y:{:.1f}, w:{:.1f}, h:{:.1f}] | Conf: {:.2f} | Pan Mag: {:.2f} | Latency: {:.2f} ms", 
                 finalRoi.x, finalRoi.y, finalRoi.width, finalRoi.height, confidence, panMag, m_lastLogicLatencyMs);

    if (m_cfg.ui.showResult || m_cfg.ui.saveVideo) {
        cv::Mat leftSide = currFrame.clone();
        cv::rectangle(leftSide, finalRoi, cv::Scalar(0, 255, 0), 2);
        
        cv::Mat rightSide;
        cv::cvtColor(motionEnergy, rightSide, cv::COLOR_GRAY2BGR);
        cv::resize(rightSide, rightSide, leftSide.size());

        if (!res.empty()) {
            cv::Mat heatmap;
            // Normalize from 32-bit float (-1 to 1) to 8-bit integer (0 to 255)
            cv::normalize(res, heatmap, 0, 255, cv::NORM_MINMAX, CV_8U);
            
            // Apply the JET color map (Blue = Low, Red = High)
            cv::applyColorMap(heatmap, heatmap, cv::COLORMAP_JET);
            
            // Define where the search window sits in original frame coordinates
            cv::Rect displayWin(
                static_cast<int>(vWin.x / scale), static_cast<int>(vWin.y / scale),
                static_cast<int>(vWin.width / scale), static_cast<int>(vWin.height / scale)
            );
            
            // Blend the heatmap into the diagnostic feed
            displayWin &= cv::Rect(0, 0, rightSide.cols, rightSide.rows);
            if (displayWin.area() > 0) {
                cv::Mat resizedHeat;
                cv::resize(heatmap, resizedHeat, displayWin.size());
                cv::Mat roi = rightSide(displayWin);
                // 70% heatmap visibility, 30% motion outline visibility
                cv::addWeighted(roi, 0.3, resizedHeat, 0.7, 0, roi);
            }
        }

        std::vector<std::string> stats = {
            "MODE: " + std::string(panMag > m_cfg.tracker.panThreshold ? "PURSUIT" : "STATIONARY"),
            "PERCEPTION: " + cv::format("%.2f ms", m_lastLogicLatencyMs),
            "CONFIDENCE: " + std::to_string(static_cast<int>(confidence * 100)) + "%",
            "PAN MAG: " + cv::format("%.2f px/f", panMag),
            "STATUS: " + std::string(detected ? "LOCKED" : "PREDICTING")
        };

        for (size_t i = 0; i < stats.size(); ++i) {
            cv::putText(rightSide, stats[i], cv::Point(20, 50 + static_cast<int>(i) * 40), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }

        // Combine diagnostic feed and save into canvas for later retrieval
        cv::hconcat(leftSide, rightSide, m_canvas);
    }

    return finalRoi;
}

} // namespace n9m