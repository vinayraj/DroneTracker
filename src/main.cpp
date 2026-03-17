#include "Config.hpp"
#include "DroneTracker.hpp"
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <chrono>
#include <filesystem>

int main(int argc, char** argv) {
    // Load Configuration
    n9m::Config cfg;
    // Note: Cast argc/argv to void to silence unused parameter warnings if not used yet
    (void)argc; (void)argv; 

    if (!n9m::sys::ConfigManager::parse(argc, argv, cfg)) {
        return EXIT_FAILURE;
    }

    // Setup Logging
    n9m::sys::ConfigManager::dump(cfg);
    if (cfg.verbose) {
        spdlog::set_level(spdlog::level::debug);
    }

    // Initialize Video Capture
    cv::VideoCapture cap(cfg.paths.inputSource);
    if (!cap.isOpened()) {
        spdlog::error("Failed to open video source: {}", cfg.paths.inputSource);
        return EXIT_FAILURE;
    }

    double fps = cap.get(cv::CAP_PROP_FPS);
    cv::Size frameSize(static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
                       static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)));

    std::string inputPath = cfg.paths.inputSource;
    std::string fileName = std::filesystem::path(inputPath).stem().string(); 
    std::string outPath = cfg.paths.resultDir + fileName + "_processed.mp4";

    // Initialize Video Writer for Side-by-Side Diagnostic (2x Width)
    cv::VideoWriter writer;
    if (cfg.ui.saveVideo) {
        // Width is doubled to accommodate the Motion Energy view
        cv::Size diagnosticSize(frameSize.width * 2, frameSize.height);
        
        writer.open(outPath, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, diagnosticSize);
        if (!writer.isOpened()) {
            spdlog::error("Could not open VideoWriter at: {}", outPath);
        } else {
            spdlog::info("Recording Diagnostic Session ({}x{}) to: {}", 
                         diagnosticSize.width, diagnosticSize.height, outPath);
        }
    }

    // Initialize Tracker
    n9m::DroneTracker tracker(cfg);
    cv::Mat frame;
    std::string windowName = cfg.projectName;

    spdlog::info("Starting tracking loop. Target Latency: {}ms.", cfg.limits.maxFrameTimeMs);

    while (cap.read(frame)) {
        // Run Tracker
        cv::Rect2d resultBox = tracker.update(frame);

        // Retrieve the internal latency from the tracker and display it on the frame
        double perceptionMs = tracker.getInternalLatency();
    
        // This text is now a true reflection of the algorithm's performance
        std::string perf = cv::format("PERCEPTION: %.2f ms", perceptionMs);
        cv::putText(frame, perf, cv::Point(30, 80), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
     
        // Generate the "Left Side" (Primary View)
        cv::Mat leftSide = frame.clone();
        int x = static_cast<int>(resultBox.x);
        int y = static_cast<int>(resultBox.y);
        int w = static_cast<int>(resultBox.width);
        int h = static_cast<int>(resultBox.height);
        
        // Draw the ROI
        cv::rectangle(leftSide, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 0), 2);
        
        // Handle Recording (Side-by-Side Reconstruction)
        if (cfg.ui.saveVideo && writer.isOpened()) {
            cv::Mat canvas = tracker.getDiagnosticCanvas(); // Get the side-by-side canvas from the tracker
            if (!canvas.empty()) writer.write(tracker.getDiagnosticCanvas()); 
        }

        if (cfg.ui.showResult) {
            cv::imshow(windowName, leftSide);
            if (cv::waitKey(1) == 'q') break;
        }
    }

    cap.release();
    if (writer.isOpened()) writer.release();
    cv::destroyAllWindows();
    spdlog::info("Session completed.");

    return EXIT_SUCCESS;
}