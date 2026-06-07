# Laser Scanner Toolkit (LSC)

Laser Scanner Toolkit is a self-contained C++17 toolkit for 3D line-laser profiler calibration, simulation, reconstruction, point-cloud processing, and measurement.

Laser Scanner Toolkit 是一个自包含的 3D 线激光轮廓仪标定与重建工具包，无需真实硬件即可完成全链路仿真、重建与精度验证。

## Features

- Camera calibration with Zhang's method
- Light-plane calibration with cross-ratio invariance
- Motion-axis calibration with PCA / differential motion
- 3D point-cloud reconstruction through ray-plane intersection
- Point-cloud processing: statistical filtering, voxel downsampling, RANSAC plane segmentation
- Measurement: AABB, OBB, step height, and grid-based volume estimation
- Full simulation environment with synthetic data and ground-truth validation
- Optional Qt5 GUI using a subprocess runner to avoid compiler ABI coupling
- Validation programs for calibration, reconstruction, repeatability, and robustness

## Quick Start

### Requirements

- CMake >= 3.16
- C++17 compiler: Visual Studio 2022, MinGW-w64, GCC, or Clang
- OpenCV >= 4.5
- Eigen3, included under `third_party/eigen`
- Qt 5.15+, optional and only required for the GUI

Set `OpenCV_DIR` if CMake cannot find OpenCV automatically:

```powershell
$env:OpenCV_DIR = "C:\path\to\opencv\build"
```

### Build On Windows

```powershell
git clone --recursive https://github.com/<your-username>/laser_scanner_toolkit.git
cd laser_scanner_toolkit
.\build.ps1
```

Or use plain CMake:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

### Build On Linux/macOS

```bash
git clone --recursive https://github.com/<your-username>/laser_scanner_toolkit.git
cd laser_scanner_toolkit
./build.sh
```

### Run

```powershell
.\build\Release\demo_full_pipeline.exe
```

For single-config generators such as NMake or Makefiles, the executable is usually:

```bash
./build/demo_full_pipeline
```

### Qt GUI On Windows

GUI lifecycle operations are consolidated in one PowerShell entry point:

```powershell
# Build the core demo first
.\build.ps1

# Build, deploy runtime dependencies, and launch the GUI
.\gui.ps1 All
```

After deployment, the application entry point is:

```text
LaserScannerToolkit.exe
```

Users can launch this EXE directly. No PowerShell or batch script is required
at runtime. Keep the generated `app/` directory beside the launcher.

Individual operations are also available:

```powershell
.\gui.ps1 Build
.\gui.ps1 Deploy
.\gui.ps1 Run
```

Set `QT5_DIR` and `MINGW_BIN` only when Qt cannot be discovered automatically.

Long validation executables are built by default. To also register them as CTest tests, configure with:

```bash
cmake -S . -B build -DLSC_REGISTER_VALIDATION_TESTS=ON
ctest --test-dir build --output-on-failure
```

### Run Tests

`test_core` is a self-contained unit test and does not require Google Test or any extra test framework. The `validate_*` executables are integration validation programs; when registered through CTest they run in smoke mode so CI stays fast and deterministic.

```bash
cmake -S . -B build -DLSC_BUILD_TESTS=ON -DLSC_REGISTER_VALIDATION_TESTS=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Run the validation executables directly for the full long-form reports:

```bash
./build/validate_calibration
./build/validate_reconstruction
./build/validate_repeatability
./build/validate_robustness
```

### Production Checks

Strict project warnings are enabled by default. CI treats warnings as errors and also runs a Linux AddressSanitizer/UBSan job.

```bash
cmake -S . -B build-debug \
  -DCMAKE_BUILD_TYPE=Debug \
  -DLSC_BUILD_TESTS=ON \
  -DLSC_REGISTER_VALIDATION_TESTS=ON \
  -DLSC_WARNINGS_AS_ERRORS=ON \
  -DLSC_ENABLE_ASAN=ON \
  -DLSC_ENABLE_UBSAN=ON
cmake --build build-debug
ctest --test-dir build-debug --output-on-failure
```

Runtime logs use the lightweight `lsc::Logger`. Set `LSC_LOG_LEVEL` to `TRACE`, `DEBUG`, `INFO`, `WARN`, `ERROR`, or `FATAL` to control verbosity.

Optional enterprise checks are available through CMake switches:

```bash
cmake -S . -B build \
  -DLSC_BUILD_BENCHMARKS=ON \
  -DLSC_BUILD_FUZZERS=ON \
  -DLSC_ENABLE_COVERAGE=ON
```

Additional optional integration points include `LSC_BUILD_PYTHON` for pybind11 bindings, `vcpkg.json` / `conanfile.py` for package managers, and `include/lsc/hal` for future hardware adapters.

## Project Structure

```text
include/lsc/         Public headers
src/core/            Core data types, IO, laser-line extraction, ray-plane math
src/calib/           Camera, light-plane, and motion-axis calibration
src/recon/           3D reconstruction
src/proc/            Point-cloud filtering, segmentation, and measurement
src/sim/             Synthetic scanner and scene generation
demo/                End-to-end and module demos
validation/          Automated validation programs
tests/               Unit tests, benchmarks, and fuzz targets
gui/                 Optional Qt5 GUI
launcher/            Stable Windows launcher source
docs/                Architecture and generated API documentation source
python/              Optional pybind11 bindings
build.ps1            Windows core build entry point
gui.ps1              Windows GUI build/deploy/run entry point
run_demo.ps1         Windows demo runner
third_party/eigen/   Eigen3 dependency
```

The `build/`, `build_gui/`, `app/`, `output/`, and root launcher EXE are
generated locally and intentionally excluded from version control. Distribute
compiled programs through GitHub Releases instead of committing binaries to
the source repository.

## Main Demo

`demo_full_pipeline` runs the complete workflow:

1. System calibration
2. Synthetic scanning and 3D reconstruction
3. Point-cloud filtering, downsampling, and plane segmentation
4. Step-height and volume measurement

Generated files are written to `output/`, including calibration YAML files, PLY point clouds, and preview images.

## License

This project is released under the MIT License. See `LICENSE` for details.
