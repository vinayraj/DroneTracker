cmake_minimum_required(VERSION 3.20)

project(DroneTracker VERSION 1.0.0 LANGUAGES CXX)

# --- Global Settings ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Compiler Options ---
if(MSVC)
    # Standard MSVC flags for high-quality code
    add_compile_options(/W4 /permissive- /utf-8)
    
    # Only apply optimizations if explicitly in Release mode 
    # This prevents conflicts with /RTC1 in Debug mode
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(/O2 /Oi /Ot /GL)
        add_link_options(/LTCG)
    endif()
else()
    # Standard GCC/Clang flags
    add_compile_options(-Wall -Wextra -Wpedantic)
    
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(-O3 -march=native -ffast-math)
    endif()
    
    # Sanitizers for Debugging (Clang/GCC only)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(-fsanitize=address -fsanitize=undefined)
        add_link_options(-fsanitize=address -fsanitize=undefined)
    endif()
endif()

# --- Dependency Management ---
find_package(OpenCV REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(CLI11 CONFIG REQUIRED)

# --- Target Definition ---
# Automatically collect all source and header files from the folders
file(GLOB_RECURSE SRC_FILES src/*.cpp)
file(GLOB_RECURSE HEADER_FILES include/*.hpp)

# Safety check: if folders aren't used yet, look in root
if(NOT SRC_FILES)
    file(GLOB SRC_FILES *.cpp)
    file(GLOB HEADER_FILES *.hpp)
endif()

add_executable(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})

# --- Compile Definitions ---
target_compile_definitions(${PROJECT_NAME} PRIVATE 
    # Silences C4996 'localtime' warnings on Windows
    _CRT_SECURE_NO_WARNINGS
    # Provides the absolute path to your source root for path resolution
    PROJECT_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)

# --- Include & Linkage ---
target_include_directories(${PROJECT_NAME} PRIVATE include .)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    ${OpenCV_LIBS}
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    CLI11::CLI11
)

# --- Asset Deployment (The "Data Sync") ---
# Ensures your videos and config are always in the build folder next to the .exe
# Uses $<TARGET_FILE_DIR> to correctly handle Debug/Release subfolders
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/config.json"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>/config.json"
    COMMENT "Deploying config.json to output directory..."
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/data"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>/data"
    COMMENT "Syncing data/ directory to output directory..."
)