#pragma once

#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace n9m {

struct Config {
    std::string projectName = "DroneIntercept_9M";
    bool verbose = false;

    struct UI {
        bool showResult = true;
        bool showDebug = false;
        bool saveVideo = true;
    } ui;

    struct Paths {
        std::string inputSource;
        std::string logDir = "logs/";
        std::string resultDir = "results/";
    } paths;

    struct Tracker {
        double processingWidth = 640.0;
        double velocityMemory = 0.3;
        double alphaStationary = 0.3;
        double alphaPursuit = 0.95;
        double panThreshold = 8.0;
        double fallbackDecay = 0.6;
        double matchThresholdNormal = 0.45;
        double matchThresholdRapid = 0.35;
        float  spatialPenaltySigma = 30.0f;
        int    searchPaddingBase = 25;
        double searchPaddingScale = 2.5;
    } tracker;

    struct GMC {
        bool enabled = true;
        bool useHomography = false;
        int maxFeatures = 500;
    } gmc;

    struct Limits {
        int maxFrameTimeMs = 100;
    } limits;

    cv::Rect2d initialRoi;
};

namespace sys {
    class ConfigManager {
    public:
        static bool parse(int argc, char** argv, Config& outConfig);
        static void dump(const Config& cfg);
    };
} 

// --- json mapping handlers ---

inline void to_json(nlohmann::json& j, const Config::UI& u) {
    j = nlohmann::json{{"show_result", u.showResult}, {"show_debug", u.showDebug}, {"save_video", u.saveVideo}};
}

inline void from_json(const nlohmann::json& j, Config::UI& u) {
    u.showResult = j.value("show_result", true);
    u.showDebug  = j.value("show_debug", false);
    u.saveVideo  = j.value("save_video", true);
}

inline void to_json(nlohmann::json& j, const Config::Paths& p) {
    j = nlohmann::json{{"input_source", p.inputSource}, {"log_dir", p.logDir}, {"result_dir", p.resultDir}};
}

inline void from_json(const nlohmann::json& j, Config::Paths& p) {
    p.inputSource = j.value("input_source", "");
    p.logDir      = j.value("log_dir", "logs/");
    p.resultDir   = j.value("result_dir", "results/");
}

inline void to_json(nlohmann::json& j, const Config::Tracker& t) {
    j = nlohmann::json{
        {"processing_width", t.processingWidth},
        {"dynamics", {
            {"velocity_memory", t.velocityMemory}, {"alpha_stationary", t.alphaStationary},
            {"alpha_pursuit", t.alphaPursuit}, {"pan_threshold", t.panThreshold}, {"fallback_decay", t.fallbackDecay}
        }},
        {"detection", {
            {"match_threshold_normal", t.matchThresholdNormal}, {"match_threshold_rapid", t.matchThresholdRapid},
            {"spatial_penalty_sigma", t.spatialPenaltySigma}, {"search_padding_base", t.searchPaddingBase}, {"search_padding_scale", t.searchPaddingScale}
        }}
    };
}

inline void from_json(const nlohmann::json& j, Config::Tracker& t) {
    t.processingWidth = j.value("processing_width", 640.0);
    if (j.contains("dynamics")) {
        auto d = j.at("dynamics");
        t.velocityMemory = d.value("velocity_memory", 0.3);
        t.alphaStationary = d.value("alpha_stationary", 0.3);
        t.alphaPursuit = d.value("alpha_pursuit", 0.95);
        t.panThreshold = d.value("pan_threshold", 8.0);
        t.fallbackDecay = d.value("fallback_decay", 0.6);
    }
    if (j.contains("detection")) {
        auto d = j.at("detection");
        t.matchThresholdNormal = d.value("match_threshold_normal", 0.45);
        t.matchThresholdRapid = d.value("match_threshold_rapid", 0.35);
        t.spatialPenaltySigma = d.value("spatial_penalty_sigma", 30.0f);
        t.searchPaddingBase = d.value("search_padding_base", 25);
        t.searchPaddingScale = d.value("search_padding_scale", 2.5);
    }
}

inline void to_json(nlohmann::json& j, const Config::GMC& g) {
    j = nlohmann::json{{"enabled", g.enabled}, {"use_homography", g.useHomography}, {"max_features", g.maxFeatures}};
}

inline void from_json(const nlohmann::json& j, Config::GMC& g) {
    g.enabled = j.value("enabled", true);
    g.useHomography = j.value("use_homography", false);
    g.maxFeatures = j.value("max_features", 500);
}

inline void to_json(nlohmann::json& j, const Config::Limits& l) {
    j = nlohmann::json{{"max_frame_time_ms", l.maxFrameTimeMs}};
}

inline void from_json(const nlohmann::json& j, Config::Limits& l) {
    l.maxFrameTimeMs = j.value("max_frame_time_ms", 100);
}

inline void to_json(nlohmann::json& j, const Config& c) {
    j = nlohmann::json{
        {"project_name", c.projectName}, {"verbose", c.verbose},
        {"ui", c.ui}, {"paths", c.paths}, {"tracker", c.tracker},
        {"gmc", c.gmc}, {"limits", c.limits},
        {"initial_roi", {{"x", c.initialRoi.x}, {"y", c.initialRoi.y}, {"width", c.initialRoi.width}, {"height", c.initialRoi.height}}}
    };
}

inline void from_json(const nlohmann::json& j, Config& c) {
    c.projectName = j.value("project_name", "DroneIntercept_9M");
    c.verbose = j.value("verbose", false);
    if (j.contains("ui")) j.at("ui").get_to(c.ui);
    if (j.contains("paths")) j.at("paths").get_to(c.paths);
    if (j.contains("tracker")) j.at("tracker").get_to(c.tracker);
    if (j.contains("gmc")) j.at("gmc").get_to(c.gmc);
    if (j.contains("limits")) j.at("limits").get_to(c.limits);
    if (j.contains("initial_roi")) {
        auto r = j.at("initial_roi");
        c.initialRoi = cv::Rect2d(r.value("x", 0.0), r.value("y", 0.0), r.value("width", 0.0), r.value("height", 0.0));
    }
}

} // namespace n9m