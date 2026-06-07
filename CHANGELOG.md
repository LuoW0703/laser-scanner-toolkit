# Changelog

## 1.0.1 (2026-06-07)

- Fixed evidence image loading when the application is installed under a path containing Chinese characters.
- Added UTF-8 paths to the GUI subprocess protocol.
- Added end-to-end evidence auditing for generated images and point clouds.
- Changed inspection verdicts to fail when evidence records are missing, empty, or unreadable.
- Cleared stale output before each run so previous files cannot produce a false pass.

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
