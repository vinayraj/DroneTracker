#include "Config.hpp"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <fstream>
#include <filesystem>

namespace n9m {
namespace sys {

bool ConfigManager::parse(int argc, char** argv, Config& outConfig) {
    // Initialize Multi-Sink Logger (Console + File)
    if (!spdlog::get("multi_sink")) {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        
        // This is the primary session log
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/tracker.log", true);
        
        spdlog::sinks_init_list sink_list = { console_sink, file_sink };
        auto logger = std::make_shared<spdlog::logger>("multi_sink", sink_list);
        spdlog::set_default_logger(logger);
    }
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");

    std::string configPath = "config.json";
    if (argc > 1) configPath = argv[1];

    std::ifstream f(configPath);
    if (!f.is_open()) {
        spdlog::error("Could not open config file: {}", configPath);
        return false;
    }

    try {
        nlohmann::json j;
        f >> j;
        outConfig = j.get<n9m::Config>();
    } catch (const std::exception& e) {
        spdlog::error("JSON Parsing Error: {}", e.what());
        return false;
    }

    std::filesystem::create_directories(outConfig.paths.logDir);
    std::filesystem::create_directories(outConfig.paths.resultDir);

    return true;
}

void ConfigManager::dump(const Config& cfg) {
    try {
        nlohmann::json j = cfg;
        std::string dumped = j.dump(4); // Format with 4-space indent

        spdlog::info("--------------------------------------------------");
        spdlog::info("INITIALIZING SESSION: {}", cfg.projectName);
        spdlog::info("CONFIGURATION PARAMETERS:");
        
        // We split the string by newlines to log it line-by-line 
        // This ensures the log file prefix appears on every line
        std::stringstream ss(dumped);
        std::string line;
        while (std::getline(ss, line)) {
            spdlog::info("  {}", line);
        }
        
        spdlog::info("--------------------------------------------------");
    } catch (const std::exception& e) {
        spdlog::error("Failed to log configuration: {}", e.what());
    }
}

} // namespace sys
} // namespace n9m