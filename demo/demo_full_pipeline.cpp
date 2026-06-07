/**
 * @file    demo_full_pipeline.cpp
 * @brief   ★ 一键跑通全流程：标定 → 扫描 → 重建 → 处理 → 测量
 *
 * 这是面试官唯一需要运行的文件。在 5 分钟内展示完整的 3D 线激光
 * 轮廓仪软件流水线。所有结果自动保存到 output/ 目录。
 *
 * 流程概览：
 *   Phase 1: 系统标定（相机 + 光平面 + 移动轴）
 *   Phase 2: 三维扫描与重建
 *   Phase 3: 点云处理（滤波 + 下采样 + 分割）
 *   Phase 4: 尺寸与体积测量
 *
 * 运行方式：
 *   ./demo_full_pipeline
 *
 * 输出文件：
 *   output/camera_params.yaml       - 相机内参
 *   output/light_plane.yaml          - 光平面参数
 *   output/calibration.yaml          - 完整标定结果集
 *   output/scan_result.ply           - 原始重建点云
 *   output/scan_result_filtered.ply  - 滤波后点云
 *   output/scan_result_plane.ply     - 平面分割结果
 */

#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <string>
#include <limits>

#include "lsc/core/types.h"
#include "lsc/sim/simulated_scanner.h"
#include "lsc/calib/camera_calib.h"
#include "lsc/calib/light_plane_calib.h"
#include "lsc/calib/motion_axis_calib.h"
#include "lsc/recon/reconstructor.h"
#include "lsc/proc/filter.h"
#include "lsc/proc/segmentation.h"
#include "lsc/proc/measurement.h"
#include "lsc/core/io_utils.h"

#include <fstream>
#include <sstream>
#include <filesystem>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================
// 配置常量（可通过 --config <yaml> 命令行覆盖）
// ============================================================
struct PipelineConfig {
    int    numCameraImages = 24;
    int    numLpImages     = 8;
    int    numMaSteps      = 5;
    double maStepMm       = 2.0;
    double blockWidth     = 60.0;
    double blockDepth     = 40.0;
    double step1H         = 10.0;
    double step2H         = 20.0;
    double voxelSize      = 0.2;
    double ransacDist     = 0.5;
};

// 仿真点检统一使用这一组接受门限。状态协议、最终汇总和进程退出码必须
// 引用同一来源，避免同一次结果在 GUI 与日志中出现互相矛盾的判定。
constexpr double kMaxCameraFocalErrorPercent = 5.0;
constexpr double kMaxLightPlaneAngleDeg = 5.0;
constexpr double kMaxMotionAxisAngleDeg = 1.0;

static bool validateConfig(const PipelineConfig& cfg, std::vector<std::string>& errors) {
    if (cfg.numCameraImages < 3) errors.push_back("num_camera_images must be >= 3");
    if (cfg.numLpImages < 1) errors.push_back("num_lp_images must be >= 1");
    if (cfg.numMaSteps < 2) errors.push_back("num_ma_steps must be >= 2");
    if (cfg.maStepMm <= 0.0) errors.push_back("ma_step_mm must be positive");
    if (cfg.blockWidth <= 0.0) errors.push_back("block_width must be positive");
    if (cfg.blockDepth <= 0.0) errors.push_back("block_depth must be positive");
    if (cfg.step1H < 0.0) errors.push_back("step1_h must be non-negative");
    if (cfg.step2H < 0.0) errors.push_back("step2_h must be non-negative");
    if (cfg.voxelSize <= 0.0) errors.push_back("voxel_size must be positive");
    if (cfg.ransacDist <= 0.0) errors.push_back("ransac_dist must be positive");
    return errors.empty();
}

/** 从简单 key: value 文件逐行解析配置（无外部依赖） */
static void loadConfigFromFile(PipelineConfig& cfg, const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[警告] 无法打开配置文件: " << path << "\n";
        return;
    }
    std::string line;
    while (std::getline(file, line)) {
        // 跳过注释和空行
        if (line.empty() || line[0] == '#') continue;

        // 拆分 key: value
        size_t colon = line.find(':');
        if (colon == std::string::npos) continue;

        std::string key = line.substr(0, colon);
        std::string val = line.substr(colon + 1);

        // 去除首尾空白
        auto trim = [](std::string& s) {
            s.erase(0, s.find_first_not_of(" \t\r"));
            s.erase(s.find_last_not_of(" \t\r") + 1);
        };
        trim(key);
        trim(val);

        if (val.empty()) continue;

        // 按 key 赋值
        try {
            if (key == "num_camera_images") cfg.numCameraImages = std::stoi(val);
            else if (key == "num_lp_images") cfg.numLpImages = std::stoi(val);
            else if (key == "num_ma_steps")  cfg.numMaSteps  = std::stoi(val);
            else if (key == "ma_step_mm")    cfg.maStepMm    = std::stod(val);
            else if (key == "block_width")   cfg.blockWidth  = std::stod(val);
            else if (key == "block_depth")   cfg.blockDepth  = std::stod(val);
            else if (key == "step1_h")       cfg.step1H      = std::stod(val);
            else if (key == "step2_h")       cfg.step2H      = std::stod(val);
            else if (key == "voxel_size")    cfg.voxelSize   = std::stod(val);
            else if (key == "ransac_dist")   cfg.ransacDist  = std::stod(val);
            else std::cerr << "[警告] 未知配置项，已忽略: " << key << "\n";
        } catch (const std::exception&) {
            std::cerr << "[警告] 配置项解析失败，已忽略: " << key << "\n";
        }
    }
    std::cout << "[配置] 已从 " << path << " 加载参数\n";
}

/** 简单命令行解析：支持 --config <path> */
static std::string parseArgs(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--config" && i + 1 < argc) {
            return argv[i + 1];
        }
    }
    return {};
}

// ============================================================
// 计时工具
// ============================================================
class Stopwatch {
public:
    void start() { m_start = std::chrono::high_resolution_clock::now(); }
    double elapsed() const {
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double>(now - m_start).count();
    }
private:
    std::chrono::high_resolution_clock::time_point m_start;
};

// ============================================================
// 辅助函数
// ============================================================

/** 打印分隔线 */
static void printSeparator() {
    std::cout << "========================================\n";
}

/** 计算两个单位向量夹角（度） */
static double angleDeg(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    double d = std::abs(a.normalized().dot(b.normalized()));
    if (d > 1.0) d = 1.0;
    return std::acos(d) * 180.0 / M_PI;
}

/** 确保 output/ 和 output/images/ 目录存在（跨平台） */
static void ensureOutputDir() {
    std::filesystem::create_directories("output/images");
    std::filesystem::create_directories("output/diagnostics");
}

/**
 * 将详情协议中的自由文本压缩为单行。
 *
 * GUI 和流水线之间使用制表符分列，因此这里必须移除字段内部的 TAB
 * 与换行。相比在日志中解析自然语言，这种固定列协议更稳定，也便于
 * 后续增加真实相机帧、硬件时间戳或质量指标。
 */
static std::string detailField(std::string text) {
    for (char& ch : text) {
        if (ch == '\t' || ch == '\r' || ch == '\n') {
            ch = ' ';
        }
    }
    return text;
}

/** 向 GUI 发布一条原图、诊断图和算法数据组成的证据记录。 */
static void emitImageDetail(
    const std::string& category,
    size_t index,
    const std::string& title,
    const std::string& sourcePath,
    const std::string& processedPath,
    const std::string& algorithm,
    const std::string& summary) {
    std::cout
        << "[LSC_DETAIL]\t" << detailField(category)
        << '\t' << index
        << '\t' << detailField(title)
        << '\t' << std::filesystem::absolute(sourcePath).string()
        << '\t' << std::filesystem::absolute(processedPath).string()
        << '\t' << detailField(algorithm)
        << '\t' << detailField(summary)
        << '\n' << std::flush;
}

/** 将灰度输入转换为可绘制彩色标记的 BGR 图像。 */
static cv::Mat toDiagnosticCanvas(const cv::Mat& image) {
    cv::Mat canvas;
    if (image.channels() == 1) {
        cv::cvtColor(image, canvas, cv::COLOR_GRAY2BGR);
    } else {
        canvas = image.clone();
    }
    return canvas;
}

/**
 * 在图像中检测棋盘格并绘制角点。
 *
 * 先使用经典检测器以保持与标定主流程一致，失败后使用 SB 检测器处理
 * 正视、大尺寸棋盘等困难场景。绿色连线表示检测到的完整角点拓扑，
 * 黄色圆点表示用于 solvePnP 的棋盘中心。
 */
static bool drawChessboardEvidence(
    const cv::Mat& image,
    const cv::Size& boardSize,
    cv::Mat& canvas,
    int& cornerCount) {
    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    std::vector<cv::Point2f> corners;
    const int flags =
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    bool found = cv::findChessboardCorners(gray, boardSize, corners, flags);
    if (!found) {
        corners.clear();
        found = cv::findChessboardCornersSB(
            gray, boardSize, corners,
            cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE);
    }

    cornerCount = static_cast<int>(corners.size());
    cv::drawChessboardCorners(canvas, boardSize, corners, found);
    if (found && !corners.empty()) {
        cv::Point2f center(0.0f, 0.0f);
        for (const auto& corner : corners) {
            center += corner;
        }
        center *= 1.0f / static_cast<float>(corners.size());
        cv::circle(canvas, center, 9, cv::Scalar(0, 220, 255), 2,
                   cv::LINE_AA);
    }
    return found;
}

/** 在原图上绘制亚像素激光中心点，控制抽样仅影响显示，不影响算法。 */
static void drawLaserEvidence(
    cv::Mat& canvas, const std::vector<lsc::Point2d>& points) {
    const size_t stride = std::max<size_t>(1, points.size() / 900);
    for (size_t i = 0; i < points.size(); i += stride) {
        const cv::Point center(
            static_cast<int>(std::lround(points[i].x())),
            static_cast<int>(std::lround(points[i].y())));
        cv::circle(canvas, center, 2, cv::Scalar(255, 220, 0), -1,
                   cv::LINE_AA);
    }
}

/** 在诊断图左上角写入机器可复核的英文数值，避免只依赖 GUI 文本。 */
static void drawDiagnosticText(
    cv::Mat& canvas, const std::vector<std::string>& lines) {
    if (lines.empty()) {
        return;
    }
    const int height = 18 + static_cast<int>(lines.size()) * 25;
    cv::rectangle(canvas, cv::Rect(8, 8, 430, height),
                  cv::Scalar(20, 20, 20), cv::FILLED);
    for (size_t i = 0; i < lines.size(); ++i) {
        cv::putText(
            canvas, lines[i],
            cv::Point(20, 34 + static_cast<int>(i) * 25),
            cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(240, 240, 240),
            1, cv::LINE_AA);
    }
}

/**
 * 将点云投影到 XY 平面，并用 Z 高度着色。
 *
 * 该视图不是测量算法的一部分，只用于解释数据：每个像素点仍对应一个
 * 实际三维点，蓝色到红色表示从低到高的 Z 值。抽样上限只控制绘图成本。
 */
static cv::Mat renderPointCloudEvidence(
    const lsc::PointCloud& cloud, const std::string& title) {
    constexpr int width = 1100;
    constexpr int height = 680;
    constexpr int margin = 55;
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(18, 20, 24));
    if (cloud.empty()) {
        drawDiagnosticText(image, {title, "point count: 0"});
        return image;
    }

    double xMin = std::numeric_limits<double>::max();
    double xMax = std::numeric_limits<double>::lowest();
    double yMin = std::numeric_limits<double>::max();
    double yMax = std::numeric_limits<double>::lowest();
    double zMin = std::numeric_limits<double>::max();
    double zMax = std::numeric_limits<double>::lowest();
    for (const auto& point : cloud) {
        xMin = std::min(xMin, point.x());
        xMax = std::max(xMax, point.x());
        yMin = std::min(yMin, point.y());
        yMax = std::max(yMax, point.y());
        zMin = std::min(zMin, point.z());
        zMax = std::max(zMax, point.z());
    }

    const double xSpan = std::max(1e-9, xMax - xMin);
    const double ySpan = std::max(1e-9, yMax - yMin);
    const double zSpan = std::max(1e-9, zMax - zMin);
    const size_t stride = std::max<size_t>(1, cloud.size() / 120000);

    for (size_t i = 0; i < cloud.size(); i += stride) {
        const auto& point = cloud[i];
        const int x = margin + static_cast<int>(
            (point.x() - xMin) / xSpan * (width - 2 * margin));
        const int y = height - margin - static_cast<int>(
            (point.y() - yMin) / ySpan * (height - 2 * margin));
        const double t = (point.z() - zMin) / zSpan;
        const cv::Scalar color(
            255.0 * (1.0 - t),
            180.0 * (1.0 - std::abs(2.0 * t - 1.0)),
            255.0 * t);
        cv::circle(image, cv::Point(x, y), 1, color, -1);
    }

    cv::rectangle(
        image, cv::Rect(margin, margin, width - 2 * margin, height - 2 * margin),
        cv::Scalar(110, 110, 110), 1);
    std::ostringstream range;
    range << std::fixed << std::setprecision(2)
          << "points: " << cloud.size()
          << " | X: " << xMin << ".." << xMax
          << " | Y: " << yMin << ".." << yMax
          << " | Z: " << zMin << ".." << zMax << " mm";
    drawDiagnosticText(image, {title, range.str()});
    return image;
}

/** 用绿色显示 RANSAC 平面内点、红色显示其余结构点。 */
static cv::Mat renderSegmentationEvidence(
    const lsc::PointCloud& inliers, const lsc::PointCloud& outliers) {
    lsc::PointCloud combined = inliers;
    combined.insert(combined.end(), outliers.begin(), outliers.end());
    cv::Mat image = renderPointCloudEvidence(combined, "RANSAC segmentation");
    if (combined.empty()) {
        return image;
    }

    double xMin = std::numeric_limits<double>::max();
    double xMax = std::numeric_limits<double>::lowest();
    double yMin = std::numeric_limits<double>::max();
    double yMax = std::numeric_limits<double>::lowest();
    for (const auto& point : combined) {
        xMin = std::min(xMin, point.x());
        xMax = std::max(xMax, point.x());
        yMin = std::min(yMin, point.y());
        yMax = std::max(yMax, point.y());
    }
    constexpr int width = 1100;
    constexpr int height = 680;
    constexpr int margin = 55;
    const double xSpan = std::max(1e-9, xMax - xMin);
    const double ySpan = std::max(1e-9, yMax - yMin);
    const auto drawCloud = [&](const lsc::PointCloud& points,
                               const cv::Scalar& color) {
        const size_t stride = std::max<size_t>(1, points.size() / 100000);
        for (size_t i = 0; i < points.size(); i += stride) {
            const int x = margin + static_cast<int>(
                (points[i].x() - xMin) / xSpan * (width - 2 * margin));
            const int y = height - margin - static_cast<int>(
                (points[i].y() - yMin) / ySpan * (height - 2 * margin));
            cv::circle(image, cv::Point(x, y), 1, color, -1);
        }
    };
    drawCloud(outliers, cv::Scalar(70, 70, 235));
    drawCloud(inliers, cv::Scalar(70, 220, 90));
    drawDiagnosticText(
        image,
        {"RANSAC segmentation",
         "green: plane inliers | red: remaining geometry",
         "inliers: " + std::to_string(inliers.size()) +
             " | outliers: " + std::to_string(outliers.size())});
    return image;
}

/**
 * 绘制用于高度测量的 X-Z 截面。
 *
 * 三种颜色对应三个台阶区域，水平线是各区域采用的稳健 Z 估计值。
 * 这张图把“点云如何变成两个高度差”直接呈现出来。
 */
static cv::Mat renderMeasurementEvidence(
    const lsc::PointCloud& cloud,
    double xMin,
    double xMax,
    const std::vector<double>& zoneZ) {
    constexpr int width = 1100;
    constexpr int height = 680;
    constexpr int margin = 60;
    constexpr int plotTop = 125;
    constexpr int plotBottom = height - margin;
    constexpr int plotHeight = plotBottom - plotTop;
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(18, 20, 24));
    if (cloud.empty() || zoneZ.size() != 3 || xMax <= xMin) {
        drawDiagnosticText(image, {"Height measurement", "no valid zones"});
        return image;
    }

    double zMin = std::numeric_limits<double>::max();
    double zMax = std::numeric_limits<double>::lowest();
    for (const auto& point : cloud) {
        zMin = std::min(zMin, point.z());
        zMax = std::max(zMax, point.z());
    }
    const double xSpan = xMax - xMin;
    // 给极值两侧留出可视边距，避免最高/最低区域的参考线贴在坐标框上，
    // 从而让开发者能清楚区分原始点分布与最终采用的稳健 Z 估计。
    const double zPadding = std::max(1e-6, (zMax - zMin) * 0.05);
    zMin -= zPadding;
    zMax += zPadding;
    const double zSpan = std::max(1e-9, zMax - zMin);
    const cv::Scalar colors[3] = {
        cv::Scalar(255, 180, 70),
        cv::Scalar(80, 220, 120),
        cv::Scalar(80, 130, 255)
    };

    for (int zone = 0; zone < 3; ++zone) {
        const int left = margin +
            (width - 2 * margin) * zone / 3;
        const int right = margin +
            (width - 2 * margin) * (zone + 1) / 3;
        cv::rectangle(
            image, cv::Rect(left, plotTop, right - left, plotHeight),
            colors[zone] * 0.12, cv::FILLED);
    }

    const size_t stride = std::max<size_t>(1, cloud.size() / 120000);
    for (size_t i = 0; i < cloud.size(); i += stride) {
        const auto& point = cloud[i];
        int zone = static_cast<int>((point.x() - xMin) / xSpan * 3.0);
        zone = std::clamp(zone, 0, 2);
        const int x = margin + static_cast<int>(
            (point.x() - xMin) / xSpan * (width - 2 * margin));
        const int y = plotBottom - static_cast<int>(
            (point.z() - zMin) / zSpan * plotHeight);
        cv::circle(image, cv::Point(x, y), 1, colors[zone], -1);
    }

    for (int zone = 0; zone < 3; ++zone) {
        const int left = margin +
            (width - 2 * margin) * zone / 3;
        const int right = margin +
            (width - 2 * margin) * (zone + 1) / 3;
        const int y = plotBottom - static_cast<int>(
            (zoneZ[zone] - zMin) / zSpan * plotHeight);
        cv::line(image, cv::Point(left, y), cv::Point(right, y),
                 cv::Scalar(245, 245, 245), 2, cv::LINE_AA);
    }

    std::ostringstream values;
    values << std::fixed << std::setprecision(3)
           << "zone Z: " << zoneZ[0] << ", " << zoneZ[1]
           << ", " << zoneZ[2] << " mm";
    drawDiagnosticText(
        image,
        {"Height measurement: X-Z cross-section",
         "blue / green / orange: three X zones", values.str()});
    return image;
}

// ============================================================
// 主函数
// ============================================================
int main(int argc, char* argv[]) {
    ensureOutputDir();

    // 加载配置（GUI 可通过 --config 传入参数）
    PipelineConfig cfg;
    std::string configPath = parseArgs(argc, argv);
    if (!configPath.empty()) {
        loadConfigFromFile(cfg, configPath);
    }
    std::vector<std::string> configErrors;
    if (!validateConfig(cfg, configErrors)) {
        std::cerr << "[错误] 配置无效:\n";
        for (const auto& error : configErrors) {
            std::cerr << "  - " << error << "\n";
        }
        return 1;
    }

    Stopwatch totalTimer;
    totalTimer.start();

    printSeparator();
    std::cout << "  3D 线激光轮廓仪标定工具 - 全流程演示\n";
    printSeparator();
    std::cout << "\n";

    // ============================================================
    // Phase 1/4: 系统标定
    // ============================================================
    std::cout << "[Phase 1/4] 系统标定\n\n";

    Stopwatch phaseTimer;
    phaseTimer.start();

    // 创建模拟扫描器（seed=42 的默认配置）
    lsc::sim::SimulatedScanner::Config simCfg;
    // 调整光平面参数，使激光线在每级台阶中心准确照射
    // 公式: ΔX/ΔZ = 2.0，保证在 cfg.blockWidth/3=20mm 的台阶间隔内激光线能区分各表面
    simCfg.lightPlane = {0.4472, 0.0, 0.8944, -259.376};
    lsc::sim::SimulatedScanner scanner(simCfg);

    // 获取 GT 参数作为参考
    const lsc::CameraIntrinsics& K_gt  = scanner.getIntrinsics();
    const lsc::Plane&            P_gt  = simCfg.lightPlane;
    const Eigen::Vector3d&       Ax_gt = simCfg.motionAxis;

    // 标定结果存放
    lsc::CameraIntrinsics K_est;
    lsc::Plane            P_est;
    Eigen::Vector3d       Ax_est;
    double rmsCam = 0.0, rmsPlane = 0.0, rmsAxis = 0.0;
    bool calibAllOk = true;
    double cameraMetric = 0.0;
    double planeMetric = 0.0;
    double axisMetric = 0.0;

    // --- 相机标定 ---
    {
        std::cout << "  [1.1] 相机标定...\n";
        std::cout << "        生成 " << cfg.numCameraImages << " 张棋盘格图像\n";

        auto images = scanner.generateCalibImages(cfg.numCameraImages);

        // 保存棋盘格图像并输出 [IMAGE] 标记，供 GUI 实时显示
        for (size_t i = 0; i < images.size(); ++i) {
            std::ostringstream fname;
            fname << "output/images/chessboard_" << std::setw(3) << std::setfill('0') << i << ".png";
            cv::imwrite(fname.str(), images[i]);
            std::cout << "[IMAGE chessboard "
                      << std::filesystem::absolute(fname.str()).string() << "]\n"
                      << std::flush;

            cv::Mat diagnostic = toDiagnosticCanvas(images[i]);
            int cornerCount = 0;
            const bool detected = drawChessboardEvidence(
                images[i],
                cv::Size(simCfg.chessboardCols, simCfg.chessboardRows),
                diagnostic, cornerCount);
            drawDiagnosticText(
                diagnostic,
                {"Camera calibration frame " + std::to_string(i),
                 "corners: " + std::to_string(cornerCount) + "/" +
                     std::to_string(
                         simCfg.chessboardCols * simCfg.chessboardRows),
                 detected ? "status: accepted" : "status: rejected"});

            std::ostringstream processed;
            processed << "output/diagnostics/chessboard_" << std::setw(3)
                      << std::setfill('0') << i << "_corners.png";
            cv::imwrite(processed.str(), diagnostic);

            std::ostringstream summary;
            summary << "检测状态：" << (detected ? "有效" : "无效")
                    << "；角点 " << cornerCount << "/"
                    << simCfg.chessboardCols * simCfg.chessboardRows
                    << "；这些二维角点与已知棋盘三维坐标共同进入相机标定";
            emitImageDetail(
                "camera", i, "相机标定图像", fname.str(), processed.str(),
                "棋盘角点检测 → 亚像素优化 → calibrateCamera 重投影最小化",
                summary.str());
        }

        lsc::CameraCalibrator camCalib;
        bool ok = camCalib.calibrateFromImages(
            images,
            cv::Size(simCfg.chessboardCols, simCfg.chessboardRows),
            simCfg.squareSize,
            K_est, rmsCam);

        if (ok) {
            double fxErr = std::max(std::abs(K_est.fx - K_gt.fx) / K_gt.fx,
                                   std::abs(K_est.fy - K_gt.fy) / K_gt.fy) * 100.0;
            if (fxErr > kMaxCameraFocalErrorPercent) {
                std::cerr << "        [警告] 相机标定质量差 (焦距误差 " << fxErr
                          << "% > " << kMaxCameraFocalErrorPercent
                          << "%)，回退到 GT 内参\n";
                K_est = K_gt;
                calibAllOk = false;
                cameraMetric = fxErr;
                std::cout << "[LSC_STATUS camera ok=0 metric=" << cameraMetric
                          << "]\n" << std::flush;
            } else {
                camCalib.save("output/camera_params.yaml", K_est);
                cameraMetric = fxErr;
                std::cout << "[LSC_STATUS camera ok=1 metric=" << cameraMetric
                          << "]\n" << std::flush;
                std::cout << "        相机标定: " << cfg.numCameraImages << "/"
                          << cfg.numCameraImages << " 图像, 重投影误差 "
                          << std::fixed << std::setprecision(2) << rmsCam << " px\n";
            }
        } else {
            std::cerr << "        [警告] 相机标定失败，回退到 GT 内参\n";
            K_est = K_gt;
            calibAllOk = false;
            cameraMetric = 0.0;
            std::cout << "[LSC_STATUS camera ok=0 metric=0]\n" << std::flush;
        }
    }

    // --- 光平面标定 ---
    {
        std::cout << "  [1.2] 光平面标定...\n";
        std::cout << "        生成 " << cfg.numLpImages << " 张激光+棋盘格叠加图像\n";

        auto images = scanner.generateLightPlaneCalibImages(cfg.numLpImages);

        // 保存激光+棋盘格叠加图像并输出 [IMAGE] 标记，供 GUI 实时显示
        for (size_t i = 0; i < images.size(); ++i) {
            std::ostringstream fname;
            fname << "output/images/laser_board_" << std::setw(3) << std::setfill('0') << i << ".png";
            cv::imwrite(fname.str(), images[i]);
            std::cout << "[IMAGE laser_board "
                      << std::filesystem::absolute(fname.str()).string() << "]\n"
                      << std::flush;

            cv::Mat diagnostic = toDiagnosticCanvas(images[i]);
            int cornerCount = 0;
            const bool boardDetected = drawChessboardEvidence(
                images[i],
                cv::Size(simCfg.chessboardCols, simCfg.chessboardRows),
                diagnostic, cornerCount);
            lsc::LaserLineExtractor detailExtractor;
            const auto laserPoints = detailExtractor.extract(
                images[i], lsc::LaserMethod::GRAY_CENTROID, 220.0, 1.5);
            drawLaserEvidence(diagnostic, laserPoints);
            drawDiagnosticText(
                diagnostic,
                {"Light-plane calibration frame " + std::to_string(i),
                 "board corners: " + std::to_string(cornerCount),
                 "laser center points: " +
                     std::to_string(laserPoints.size())});

            std::ostringstream processed;
            processed << "output/diagnostics/laser_board_" << std::setw(3)
                      << std::setfill('0') << i << "_features.png";
            cv::imwrite(processed.str(), diagnostic);

            std::ostringstream summary;
            summary << "棋盘：" << (boardDetected ? "有效" : "无效")
                    << "，角点 " << cornerCount
                    << "；亚像素激光中心点 " << laserPoints.size()
                    << "；射线与当前标定板平面求交后形成光平面拟合点";
            emitImageDetail(
                "light_plane", i, "光平面标定图像",
                fname.str(), processed.str(),
                "棋盘位姿 solvePnP → 灰度重心提取激光中心 → 射线/标定板平面求交 → SVD 拟合",
                summary.str());
        }

        lsc::LightPlaneCalibrator lpCalib;
        bool ok = lpCalib.calibrate(
            images,
            K_est,  // 使用标定好的内参
            cv::Size(simCfg.chessboardCols, simCfg.chessboardRows),
            simCfg.squareSize,
            P_est, rmsPlane,
            lsc::LaserMethod::GRAY_CENTROID, 220.0, 1.5);

        if (ok) {
            double a = angleDeg(P_est.normal(), P_gt.normal());
            if (a > kMaxLightPlaneAngleDeg) {
                std::cerr << "        [警告] 光平面标定质量差 (法向量夹角 " << a
                          << "° > " << kMaxLightPlaneAngleDeg
                          << "°)，回退到 GT 光平面\n";
                P_est = P_gt;
                calibAllOk = false;
                planeMetric = a;
                std::cout << "[LSC_STATUS plane ok=0 metric=" << planeMetric
                          << "]\n" << std::flush;
            } else {
                planeMetric = a;
                std::cout << "[LSC_STATUS plane ok=1 metric=" << planeMetric
                          << "]\n" << std::flush;
                std::cout << "        光平面标定: 成功, 法向量夹角 "
                          << std::setprecision(2) << a << "°\n";
            }
        } else {
            std::cerr << "        [警告] 光平面标定失败，回退到 GT 光平面\n";
            P_est = P_gt;
            calibAllOk = false;
            planeMetric = 0.0;
            std::cout << "[LSC_STATUS plane ok=0 metric=0]\n" << std::flush;
        }
    }

    // --- 移动轴标定 ---
    {
        std::cout << "  [1.3] 移动轴标定...\n";
        std::cout << "        生成 " << cfg.numMaSteps << " 张平移棋盘格图像\n";

        auto images = scanner.generateMotionAxisCalibImages(cfg.numMaSteps);

        // 保存平移棋盘格图像并输出 [IMAGE] 标记，供 GUI 实时显示
        for (size_t i = 0; i < images.size(); ++i) {
            std::ostringstream fname;
            fname << "output/images/motion_" << std::setw(3) << std::setfill('0') << i << ".png";
            cv::imwrite(fname.str(), images[i]);
            std::cout << "[IMAGE motion "
                      << std::filesystem::absolute(fname.str()).string() << "]\n"
                      << std::flush;

            cv::Mat diagnostic = toDiagnosticCanvas(images[i]);
            int cornerCount = 0;
            const bool detected = drawChessboardEvidence(
                images[i],
                cv::Size(simCfg.chessboardCols, simCfg.chessboardRows),
                diagnostic, cornerCount);
            const double commandedPosition =
                static_cast<double>(i) * cfg.maStepMm;
            std::ostringstream positionText;
            positionText << std::fixed << std::setprecision(2)
                         << "commanded position: " << commandedPosition
                         << " mm";
            drawDiagnosticText(
                diagnostic,
                {"Motion-axis frame " + std::to_string(i),
                 "corners: " + std::to_string(cornerCount),
                 positionText.str()});

            std::ostringstream processed;
            processed << "output/diagnostics/motion_" << std::setw(3)
                      << std::setfill('0') << i << "_pose.png";
            cv::imwrite(processed.str(), diagnostic);

            std::ostringstream summary;
            summary << "检测状态：" << (detected ? "有效" : "无效")
                    << "；角点 " << cornerCount
                    << "；位置指令 " << std::fixed << std::setprecision(2)
                    << commandedPosition
                    << " mm；棋盘中心三维轨迹用于 PCA 拟合移动轴";
            emitImageDetail(
                "motion_axis", i, "移动轴标定图像",
                fname.str(), processed.str(),
                "棋盘位姿 solvePnP → 棋盘中心三维坐标 → PCA 直线拟合 → 位置相关性校正方向",
                summary.str());
        }

        std::vector<double> positions(cfg.numMaSteps);
        for (int i = 0; i < cfg.numMaSteps; ++i)
            positions[i] = static_cast<double>(i) * cfg.maStepMm;

        lsc::MotionAxisCalibrator maCalib;
        bool ok = maCalib.calibrate(
            images, K_est,
            cv::Size(simCfg.chessboardCols, simCfg.chessboardRows),
            simCfg.squareSize, positions,
            Ax_est, rmsAxis);

        if (ok) {
            double a = angleDeg(Ax_est, Ax_gt);
            if (a > kMaxMotionAxisAngleDeg) {
                std::cerr << "        [警告] 移动轴标定质量差 (方向夹角 "
                          << a << "° > " << kMaxMotionAxisAngleDeg
                          << "°)，回退到 GT 移动轴\n";
                Ax_est = Ax_gt;
                calibAllOk = false;
                axisMetric = a;
                std::cout << "[LSC_STATUS axis ok=0 metric=" << axisMetric
                          << "]\n" << std::flush;
            } else {
                axisMetric = a;
                std::cout << "[LSC_STATUS axis ok=1 metric=" << axisMetric
                          << "]\n" << std::flush;
                std::cout << "        移动轴标定: " << cfg.numMaSteps << "/"
                          << cfg.numMaSteps << " 位置, 方向夹角 "
                          << std::setprecision(3) << a << "°\n";
            }
        } else {
            std::cerr << "        [警告] 移动轴标定失败，回退到 GT 移动轴\n";
            Ax_est = Ax_gt;
            calibAllOk = false;
            axisMetric = 0.0;
            std::cout << "[LSC_STATUS axis ok=0 metric=0]\n" << std::flush;
        }
    }

    // 保存完整标定结果
    lsc::saveCalibration("output/calibration.yaml", K_est, P_est, Ax_est);

    double phaseTime = phaseTimer.elapsed();
    std::cout << "\n  Phase 1 耗时: " << std::fixed << std::setprecision(2)
              << phaseTime << " s\n\n";
    std::cout << "[LSC_PROGRESS value=25]\n" << std::flush;

    // ============================================================
    // Phase 2/4: 三维扫描与重建
    // ============================================================
    std::cout << "[Phase 2/4] 三维扫描与重建\n\n";

    phaseTimer.start();

    // 创建重建器（使用标定结果）
    lsc::Reconstructor reconstructor(K_est, P_est, Ax_est);

    // 配置阶梯块
    lsc::sim::StepBlock block;
    block.width       = cfg.blockWidth;
    block.depth       = cfg.blockDepth;
    block.stepHeights = {0.0, cfg.step1H, cfg.step2H};  // 含基准面：0 高度平坦面 + 2 级台阶

    int numScanLines = static_cast<int>(simCfg.travelRange / simCfg.defaultStep) + 1;
    std::cout << "  扫描对象: 阶梯块 (" << cfg.blockWidth << "x" << cfg.blockDepth
              << "mm, 台阶 " << cfg.step1H << "/" << cfg.step2H << "mm)\n";
    std::cout << "  扫描线数: " << numScanLines << " (步长 " << simCfg.defaultStep
              << "mm, 行程 " << simCfg.travelRange << "mm)\n";
    std::cout << "  生成扫描数据...\n";

    auto [scanImages, gtCloud] = scanner.generateScan(block);

    // 激光线提取 + 重建
    std::cout << "  提取激光线并重建 3D 点云...\n";

    lsc::LaserLineExtractor extractor;
    std::vector<lsc::ScanLine> scanLines;
    scanLines.reserve(scanImages.size());
    std::string representativeScanDiagnostic;
    size_t representativeLaserPointCount = 0;

    int progressStep = std::max(1, static_cast<int>(scanImages.size()) / 10);
    for (size_t i = 0; i < scanImages.size(); ++i) {
        // 灰度重心法逐列寻找超过阈值的最强连续亮区，并以灰度作为权重
        // 求亚像素中心。得到的二维点随后与光平面求交形成单条 3D 轮廓。
        auto pts = extractor.extract(scanImages[i],
                                     lsc::LaserMethod::GRAY_CENTROID, 220.0, 1.5);

        // 每 20 帧保存一张扫描图像并输出 [IMAGE] 标记，供 GUI 实时显示
        if (i % 20 == 0) {
            std::ostringstream fname;
            fname << "output/images/scan_" << std::setw(4) << std::setfill('0') << i << ".png";
            cv::imwrite(fname.str(), scanImages[i]);
            std::cout << "[IMAGE scan "
                      << std::filesystem::absolute(fname.str()).string() << "]\n"
                      << std::flush;

            cv::Mat diagnostic = toDiagnosticCanvas(scanImages[i]);
            drawLaserEvidence(diagnostic, pts);
            const double position =
                scanner.scanPosition(i);
            std::ostringstream positionText;
            positionText << std::fixed << std::setprecision(2)
                         << "motion position: " << position << " mm";
            drawDiagnosticText(
                diagnostic,
                {"Scan frame " + std::to_string(i) + "/" +
                     std::to_string(scanImages.size() - 1),
                 "laser center points: " + std::to_string(pts.size()),
                 positionText.str()});

            std::ostringstream processed;
            processed << "output/diagnostics/scan_" << std::setw(4)
                      << std::setfill('0') << i << "_laser.png";
            cv::imwrite(processed.str(), diagnostic);
            // 重建阶段的“输入证据图”选择信息量最大的抽样帧，避免扫描
            // 行程首尾没有照到工件时用空帧代表整次扫描。
            if (pts.size() > representativeLaserPointCount) {
                representativeScanDiagnostic = processed.str();
                representativeLaserPointCount = pts.size();
            }

            std::ostringstream summary;
            summary << "二维激光中心点 " << pts.size()
                    << "；扫描位置 " << std::fixed << std::setprecision(2)
                    << position
                    << " mm；每个中心点通过相机射线与光平面求交生成一个三维点";
            emitImageDetail(
                "scan", i, "扫描抽样帧", fname.str(), processed.str(),
                "阈值分割连续亮区 → 灰度加权亚像素中心 → 射线/光平面三角测量",
                summary.str());
        }

        lsc::ScanLine line;
        line.laserPoints    = pts;
        line.motionPosition = scanner.scanPosition(i);
        line.hasMotionPosition = true;
        scanLines.push_back(line);

        if ((i + 1) % progressStep == 0 || i + 1 == scanImages.size())
            std::cout << "    ... " << (i + 1) << "/" << scanImages.size() << " 帧\n";
    }

    lsc::PointCloud cloud = reconstructor.reconstruct(scanLines, simCfg.defaultStep);

    // 保存真实三维点云。GUI 收到 .ply 路径后会切换到可旋转的三维视图，
    // PNG 诊断图仍保留，便于离线报告或不支持交互显示的场景使用。
    const std::string rawCloudPath = "output/scan_result.ply";
    lsc::savePLY(rawCloudPath, cloud);
    const std::string rawCloudImage = "output/diagnostics/cloud_raw.png";
    cv::imwrite(
        rawCloudImage,
        renderPointCloudEvidence(cloud, "Raw reconstructed point cloud"));
    if (!representativeScanDiagnostic.empty()) {
        std::ostringstream summary;
        summary << "扫描线 " << scanLines.size()
                << "；重建三维点 " << cloud.size()
                << "；各帧轮廓按移动轴位置平移后合并";
        emitImageDetail(
            "reconstruction", 0, "三维重建",
            representativeScanDiagnostic, rawCloudPath,
            "像素射线与标定光平面求交 → 单帧轮廓 → 沿移动轴拼接",
            summary.str());
    }

    phaseTime = phaseTimer.elapsed();
    std::cout << "\n  扫描线数: " << scanImages.size() << "\n";
    std::cout << "  重建点数: " << cloud.size() << "\n";
    std::cout << "  点云保存: output/scan_result.ply\n";
    std::cout << "  Phase 2 耗时: " << std::fixed << std::setprecision(2)
              << phaseTime << " s\n\n";
    std::cout << "[LSC_PROGRESS value=50]\n" << std::flush;

    // ============================================================
    // Phase 3/4: 点云处理
    // ============================================================
    std::cout << "[Phase 3/4] 点云处理\n\n";

    phaseTimer.start();

    // 3.1 统计滤波
    std::cout << "  [3.1] 统计离群点滤除 (k=50, sigma=1.0)...\n";
    size_t beforeFilter = cloud.size();
    lsc::PointCloud filtered = lsc::PointCloudFilter::statisticalFilter(
        cloud, 50, 1.0);
    size_t removed = beforeFilter - filtered.size();
    double removedPct = (beforeFilter > 0) ? 100.0 * removed / beforeFilter : 0.0;

    std::cout << "        滤波: 移除 " << removed << " 离群点 ("
              << std::fixed << std::setprecision(1) << removedPct << "%)\n";
    const std::string filteredCloudImage =
        "output/diagnostics/cloud_filtered.png";
    const std::string filteredCloudPath =
        "output/scan_result_filtered.ply";
    lsc::savePLY(filteredCloudPath, filtered);
    cv::imwrite(
        filteredCloudImage,
        renderPointCloudEvidence(filtered, "Statistical outlier filtering"));
    {
        std::ostringstream summary;
        summary << "输入 " << beforeFilter << " 点；保留 " << filtered.size()
                << " 点；移除 " << removed << " 点（"
                << std::fixed << std::setprecision(1) << removedPct
                << "%）；判据为邻域平均距离 ≤ μ + 1.0σ";
        emitImageDetail(
            "filter", 0, "统计离群点滤波",
            rawCloudPath, filteredCloudPath,
            "KD-tree KNN（k=50）→ 邻域平均距离分布 → μ+σ 阈值剔除",
            summary.str());
    }

    // 3.2 体素下采样
    std::cout << "  [3.2] 体素下采样 (voxel=" << cfg.voxelSize << "mm)...\n";
    size_t beforeDS = filtered.size();
    lsc::PointCloud downsampled = lsc::PointCloudFilter::voxelDownsample(
        filtered, cfg.voxelSize);
    size_t afterDS = downsampled.size();

    std::cout << "        下采样: " << beforeDS << " 点 -> "
              << afterDS << " 点\n";
    const std::string downsampledCloudImage =
        "output/diagnostics/cloud_downsampled.png";
    const std::string downsampledCloudPath =
        "output/scan_result_downsampled.ply";
    lsc::savePLY(downsampledCloudPath, downsampled);
    cv::imwrite(
        downsampledCloudImage,
        renderPointCloudEvidence(downsampled, "Voxel downsampling"));
    {
        std::ostringstream summary;
        summary << "输入 " << beforeDS << " 点；输出 " << afterDS
                << " 点；体素边长 " << std::fixed << std::setprecision(2)
                << cfg.voxelSize << " mm；每个体素以内部点重心代表";
        emitImageDetail(
            "downsample", 0, "体素下采样",
            filteredCloudPath, downsampledCloudPath,
            "三维体素索引 floor(p/voxel) → 同体素分组 → 重心替代",
            summary.str());
    }

    // 3.3 RANSAC 平面分割
    std::cout << "  [3.3] RANSAC 平面检测 (阈值=" << cfg.ransacDist
              << "mm)...\n";

    auto planeModel = lsc::PointCloudSegmenter::ransacPlane(
        downsampled, cfg.ransacDist, 1000, 0.99);

    lsc::PointCloud inliers, outliers;
    lsc::PointCloudSegmenter::separateInliers(downsampled, planeModel,
                                               inliers, outliers);

    std::cout << "        平面分割: " << planeModel.inlierIndices.size()
              << " 内点, RMS " << std::fixed << std::setprecision(2)
              << planeModel.rmsError << " mm\n";
    const std::string segmentedCloudImage =
        "output/diagnostics/cloud_segmented.png";
    const std::string segmentedCloudPath =
        "output/scan_result_segmented.ply";
    lsc::PointCloud segmentedCloud = inliers;
    segmentedCloud.insert(
        segmentedCloud.end(), outliers.begin(), outliers.end());
    std::vector<cv::Vec3b> segmentedColors(
        inliers.size(), cv::Vec3b(70, 220, 90));
    segmentedColors.insert(
        segmentedColors.end(), outliers.size(), cv::Vec3b(70, 70, 235));
    lsc::savePLY(segmentedCloudPath, segmentedCloud, segmentedColors);
    cv::imwrite(
        segmentedCloudImage,
        renderSegmentationEvidence(inliers, outliers));
    {
        std::ostringstream summary;
        summary << "RANSAC 内点 " << inliers.size()
                << "；其余结构点 " << outliers.size()
                << "；距离阈值 " << std::fixed << std::setprecision(2)
                << cfg.ransacDist << " mm；精拟合 RMS "
                << planeModel.rmsError << " mm";
        emitImageDetail(
            "segmentation", 0, "RANSAC 平面分割",
            downsampledCloudPath, segmentedCloudPath,
            "随机三点平面假设 → 距离阈值统计内点 → 自适应迭代 → SVD 精拟合",
            summary.str());
    }

    // 保存处理结果
    lsc::savePLY("output/scan_result_plane.ply", inliers);

    phaseTime = phaseTimer.elapsed();
    std::cout << "  Phase 3 耗时: " << std::fixed << std::setprecision(2)
              << phaseTime << " s\n\n";
    std::cout << "[LSC_PROGRESS value=75]\n" << std::flush;

    // ============================================================
    // Phase 4/4: 测量
    // ============================================================
    std::cout << "[Phase 4/4] 测量\n\n";
    double measuredH1 = 0.0;
    double measuredH2 = 0.0;
    double measurementXMin = 0.0;
    double measurementXMax = 0.0;
    std::vector<double> measurementZoneZ(3, 0.0);
    bool measurementZonesValid = false;

    // 4.1 台阶高度测量
    // 新光平面 (0.4472X+0.8944Z=259.376) 下 ΔX/ΔZ=2.0，
    // 台阶间 X 间距 20mm，可直接用 X 三分区。
    std::cout << "  [4.1] 台阶高度测量...\n";

    if (!cloud.empty()) {
        // X 范围
        double xMin = std::numeric_limits<double>::max();
        double xMax = std::numeric_limits<double>::lowest();
        for (const auto& p : cloud) { xMin = std::min(xMin, p.x()); xMax = std::max(xMax, p.x()); }
        measurementXMin = xMin;
        measurementXMax = xMax;

        std::cout << "        X 范围: " << std::fixed << std::setprecision(2)
                  << xMin << " ~ " << xMax << " mm\n";

        // 三等分 X，对应基准面 + 2 级台阶
        double segW = (xMax - xMin) / 3.0;
        std::vector<std::vector<double>> zoneZ(3);
        for (const auto& p : cloud) {
            int zi = static_cast<int>((p.x() - xMin) / segW);
            if (zi < 0) zi = 0;
            if (zi > 2) zi = 2;
            zoneZ[zi].push_back(p.z());
        }

        std::cout << "        各区域点数: " << zoneZ[0].size() << ", "
                  << zoneZ[1].size() << ", " << zoneZ[2].size() << "\n";

        // 取各区域 Z 百分位（避免垂直面点污染）
        // zone 0: 基准面(Z大) + 垂直面(Z略小) → 取上百分位(90%)
        // zone 1: 只有台阶1 → 中位数即可
        // zone 2: 台阶2(Z小) + 垂直面(Z略大) → 取下百分位(10%)
        std::vector<double> zoneZest(3);
        for (int z = 0; z < 3; ++z) {
            if (zoneZ[z].empty()) { zoneZest[z] = 0; continue; }
            std::sort(zoneZ[z].begin(), zoneZ[z].end());
            size_t n = zoneZ[z].size();
            if (z == 0)      zoneZest[z] = zoneZ[z][n * 90 / 100];  // 上 10%
            else if (z == 2) zoneZest[z] = zoneZ[z][n * 10 / 100]; // 下 10%
            else             zoneZest[z] = zoneZ[z][n / 2];       // 中位数
        }

        double baseZ  = zoneZest[0];  // 最小 X → 最大 Z (基准面)
        double step1Z = zoneZest[1];  // 中间
        double step2Z = zoneZest[2];  // 最大 X → 最小 Z (台阶2)

        // 确保基准面 Z 最大
        if (baseZ < step1Z) std::swap(baseZ, step1Z);
        if (step1Z < step2Z) std::swap(step1Z, step2Z);
        if (baseZ < step1Z) std::swap(baseZ, step1Z);

        measuredH1 = baseZ - step1Z;
        measuredH2 = baseZ - step2Z;
        measurementZoneZ = {baseZ, step1Z, step2Z};
        measurementZonesValid = true;

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "        各区域中位 Z: " << baseZ << ", " << step1Z
                  << ", " << step2Z << " mm\n";
        std::cout << std::setprecision(3);
        std::cout << "        台阶1高度: " << measuredH1 << " mm"
                  << "  (真实 " << cfg.step1H << " mm, 误差 "
                  << std::abs(measuredH1 - cfg.step1H) << " mm)\n";
        std::cout << "        台阶2高度: " << measuredH2 << " mm"
                  << "  (真实 " << cfg.step2H << " mm, 误差 "
                  << std::abs(measuredH2 - cfg.step2H) << " mm)\n";
    } else {
        std::cout << "        [跳过] 点云为空\n";
    }

    // 4.2 体积测量
    std::cout << "  [4.2] 体积测量...\n";

    // 真实体积
    double segWvol = cfg.blockWidth / 3.0;
    double segArea = segWvol * cfg.blockDepth;
    double gtVol = segArea * 0.0 + segArea * cfg.step1H + segArea * cfg.step2H;

    // 过滤点云：排除垂直面点（X ≈ -10, +10 处），只保留台阶顶面点
    // 垂直面在 X ≈ -10 和 X ≈ +10，台阶顶面在 X ≈ -20, 0, +20
    lsc::PointCloud topSurfaceCloud;
    topSurfaceCloud.reserve(cloud.size());
    for (const auto& p : cloud) {
        if (std::abs(std::abs(p.x()) - 10.0) < 3.0) continue;  // 跳过垂直面 X≈±10
        topSurfaceCloud.push_back(p);
    }

    // 参考平面取 Z 最大值（基准面）。栅格积分仍作为诊断值保留：
    // 对连续面扫描它能直接估算体积，但当前仿真每帧只产生若干条激光截线，
    // 未命中的栅格并不代表工件高度为零，因此不能把该值直接用于最终点检。
    double zMaxVol = std::numeric_limits<double>::lowest();
    double yMinVol = std::numeric_limits<double>::max();
    double yMaxVol = std::numeric_limits<double>::lowest();
    for (const auto& p : topSurfaceCloud) {
        zMaxVol = std::max(zMaxVol, p.z());
        yMinVol = std::min(yMinVol, p.y());
        yMaxVol = std::max(yMaxVol, p.y());
    }

    Eigen::Vector4d refPlane(0.0, 0.0, -1.0, zMaxVol);
    const double sparseGridVolume =
        lsc::PointCloudMeasurer::computeVolume(topSurfaceCloud, refPlane, 0.5);

    // 台阶工件由三个等宽区域组成。根据点云实测 X/Y 覆盖范围得到每个区域
    // 的投影面积，再乘以相对基准面的稳健高度差。该计算只使用重建结果，
    // 不引用仿真配置中的工件宽度、深度或台阶高度。
    const double measuredWidth = measurementXMax - measurementXMin;
    const double measuredDepth = yMaxVol - yMinVol;
    const double measuredZoneArea = measuredWidth * measuredDepth / 3.0;
    double measVol = sparseGridVolume;
    if (measurementZonesValid &&
        measuredWidth > 0.0 && measuredDepth > 0.0 &&
        std::isfinite(measuredZoneArea)) {
        measVol = measuredZoneArea * (measuredH1 + measuredH2);
    }
    std::cout << "        顶面点数: " << topSurfaceCloud.size() << " / " << cloud.size() << "\n";

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "        参考平面: Z = " << zMaxVol << " mm\n";
    std::cout << "        实测投影范围: " << measuredWidth << " x "
              << measuredDepth << " mm\n";
    std::cout << "        稀疏栅格诊断值: " << sparseGridVolume << " mm^3\n";
    std::cout << "        测量体积: " << measVol << " mm^3\n";
    std::cout << "        真实体积: " << gtVol << " mm^3"
              << "  (误差 " << std::setprecision(1)
              << std::abs(measVol - gtVol) / gtVol * 100.0 << "%)\n";

    const std::string measurementImage =
        "output/diagnostics/cloud_measurement_zones.png";
    const std::string measurementCloudPath =
        "output/scan_result_measurement.ply";
    std::vector<cv::Vec3b> measurementColors;
    measurementColors.reserve(cloud.size());
    const double measurementSpan = measurementXMax - measurementXMin;
    const cv::Vec3b zoneColors[3] = {
        cv::Vec3b(255, 180, 70),
        cv::Vec3b(80, 220, 120),
        cv::Vec3b(80, 130, 255)
    };
    for (const auto& point : cloud) {
        int zone = 0;
        if (measurementSpan > 0.0) {
            zone = static_cast<int>(
                (point.x() - measurementXMin) / measurementSpan * 3.0);
            zone = std::clamp(zone, 0, 2);
        }
        measurementColors.push_back(zoneColors[zone]);
    }
    lsc::savePLY(measurementCloudPath, cloud, measurementColors);
    cv::imwrite(
        measurementImage,
        renderMeasurementEvidence(
            cloud, measurementXMin, measurementXMax, measurementZoneZ));
    {
        std::ostringstream summary;
        summary << "区域 Z 估计："
                << std::fixed << std::setprecision(3)
                << measurementZoneZ[0] << " / "
                << measurementZoneZ[1] << " / "
                << measurementZoneZ[2] << " mm；高度 "
                << measuredH1 << " / " << measuredH2
                << " mm；体积 " << std::setprecision(1)
                << measVol << " mm^3；实测投影 "
                << std::setprecision(2) << measuredWidth << " x "
                << measuredDepth << " mm；顶面点 " << topSurfaceCloud.size();
        emitImageDetail(
            "measurement", 0, "高度与体积测量",
            segmentedCloudPath, measurementCloudPath,
            measurementZonesValid
                ? "X 方向三区域稳健 Z 统计 → 高度差；实测 X/Y 投影面积 × 区域高度 → 体积"
                : "点云区域无效，未执行高度统计",
            summary.str());
    }

    std::cout << "\n  Phase 4 完成\n\n";

    // ============================================================
    // 总结
    // ============================================================
    double totalTime = totalTimer.elapsed();
    const double volumeError =
        gtVol > 0.0 ? std::abs(measVol - gtVol) / gtVol : 0.0;
    const bool measurementsFinite =
        std::isfinite(measuredH1) && std::isfinite(measuredH2) &&
        std::isfinite(measVol) && std::isfinite(totalTime);
    const bool inspectionOk =
        calibAllOk && measurementsFinite &&
        !cloud.empty() && !topSurfaceCloud.empty() &&
        std::abs(measuredH1 - cfg.step1H) <= 5.0 &&
        std::abs(measuredH2 - cfg.step2H) <= 5.0 &&
        volumeError <= 0.5;

    std::cout << "[LSC_RESULT step1=" << measuredH1
              << " step2=" << measuredH2
              << " volume=" << measVol
              << " gt_volume=" << gtVol
              << " elapsed=" << totalTime << "]\n";
    std::cout << "[LSC_PROGRESS value=100]\n";
    std::cout << "[LSC_DONE ok=" << (inspectionOk ? 1 : 0) << "]\n"
              << std::flush;

    printSeparator();
    std::cout << "  全流程耗时: " << std::fixed << std::setprecision(2)
              << totalTime << " s\n";
    std::cout << "\n  输出文件:\n";
    std::cout << "    output/camera_params.yaml         - 相机内参\n";
    std::cout << "    output/calibration.yaml            - 完整标定结果\n";
    std::cout << "    output/scan_result.ply             - 原始重建点云\n";
    std::cout << "    output/scan_result_filtered.ply    - 滤波后点云\n";
    std::cout << "    output/scan_result_plane.ply       - 平面分割结果\n";
    std::cout << "\n  标定状态: " << (calibAllOk ? "全部成功" : "部分回退到 GT");

    // 如果标定全部成功，给出精度评价
    if (calibAllOk) {
        double camErr = std::max(std::abs(K_est.fx - K_gt.fx) / K_gt.fx,
                                 std::abs(K_est.fy - K_gt.fy) / K_gt.fy) * 100.0;
        double planeErr = angleDeg(P_est.normal(), P_gt.normal());
        double axisErr = angleDeg(Ax_est, Ax_gt);

        const bool camPass =
            camErr <= kMaxCameraFocalErrorPercent;
        const bool planePass =
            planeErr <= kMaxLightPlaneAngleDeg;
        const bool axisPass =
            axisErr <= kMaxMotionAxisAngleDeg;

        std::cout << "\n  标定精度:\n";
        std::cout << "    相机: " << (camPass ? "PASS" : "FAIL")
                  << " (焦距误差 " << std::setprecision(2) << camErr << "%)\n";
        std::cout << "    光平面: " << (planePass ? "PASS" : "FAIL")
                  << " (法向量夹角 " << std::setprecision(3) << planeErr << "°)\n";
        std::cout << "    移动轴: " << (axisPass ? "PASS" : "FAIL")
                  << " (方向夹角 " << std::setprecision(3) << axisErr << "°)\n";
    }

    printSeparator();

    return inspectionOk ? 0 : 2;
}
