# Changelog

## 1.0.0 (2026-06-06)

- Initial public release.
- Full simulation pipeline: calibration, reconstruction, processing, and measurement.
- Core algorithms: Zhang camera calibration, cross-ratio light-plane calibration, PCA motion-axis calibration, ray-plane reconstruction, RANSAC plane segmentation, and point-cloud measurement.
- Optional Qt5 GUI using a subprocess architecture.
- Cross-platform CMake build support for Windows, Linux, and macOS.
- GitHub Actions CI with lightweight unit tests and validation smoke tests.
- Strict compiler warnings, optional warnings-as-errors, and sanitizer build options.
- Lightweight structured logging with `LSC_LOG_LEVEL`.
- Safer point-cloud file loading with file-size, point-count, and finite-value checks.
- KD-tree accelerated statistical filtering.
- Optional fuzzing, coverage, benchmark, clang-tidy, Doxygen, Python-binding, package-manager, and release workflows.
- Hardware abstraction and visualization extension points.
