# DroneTracker 🛰️

An ultra-low latency (<10ms) Computer Vision pipeline designed for real-time drone tracking and interception analytics. This system utilizes Global Motion Compensation (GMC), adaptive search dynamics, and sub-pixel refinement to maintain a high-fidelity lock on aerial targets.

## 🚀 Technical Highlights

* **KLT-based GMC:** Background motion estimation using High-Pyramid Lucas-Kanade Optical Flow to cancel out ego-motion.
* **Adaptive Perception:** Dynamically scales search padding and spatial penalty sigmas based on camera pan magnitude.
* **Sub-pixel Accuracy:** Parabolic interpolation on the correlation surface for precision localization.
* **Unified Logging:** Multi-sink logging that mirrors system configuration and real-time telemetry to the console and indicative `.log` files.
* **Vcpkg Integration:** Streamlined dependency management for cross-platform compatibility.

---

## 🛠 Prerequisites

This project manages C++ dependencies via **vcpkg**. 

### 1. Install Dependencies
Run the following from your terminal:

```bash
vcpkg install opencv4[ffmpeg] spdlog nlohmann-json

# Create a build directory
mkdir build
cd build

# Configure (Replace [vcpkg_root] with your actual path, e.g., C:/vcpkg)
cmake .. -DCMAKE_TOOLCHAIN_FILE=[vcpkg_root]/scripts/buildsystems/vcpkg.cmake

# Build the project
cmake --build . --config Release

# Run the tracker. All details for the tracker are in config.json
./Release/DroneTracker.exe