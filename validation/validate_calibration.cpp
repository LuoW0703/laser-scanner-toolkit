/**
 * @file    validate_calibration.cpp
 * @brief   标定精度验证：对比 SimulatedScanner 的 ground truth 与标定结果
 *
 * 流程：
 *   1. 创建 SimulatedScanner，获取所有 ground truth (K_gt, Plane_gt, Axis_gt)
 *   2. 调用 SimulatedScanner 生成标定数据
 *   3. 用 CameraCalibrator 标定得到 K_est
 *   4. 用 LightPlaneCalibrator 标定得到 Plane_est
 *   5. 用 MotionAxisCalibrator 标定得到 Axis_est
 *   6. 逐项对比并输出报告
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <vector>

#include "lsc/core/types.h"
#include "lsc/sim/simulated_scanner.h"
#include "lsc/calib/camera_calib.h"
#include "lsc/calib/light_plane_calib.h"
#include "lsc/calib/motion_axis_calib.h"
#include "lsc/core/io_utils.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================
// 判定阈值（写死在代码中）
// ============================================================
constexpr double TH_FXFY_PCT       = 0.5;   // fx/fy 误差 < 0.5% → PASS
constexpr double TH_CXCY_PX        = 2.0;   // cx/cy 误差 < 2px → PASS
constexpr double TH_NORMAL_ANGLE   = 0.5;   // 法向量夹角 < 0.5° → PASS
constexpr double TH_D_MM           = 1.0;   // D 值误差 < 1mm → PASS
constexpr double TH_AXIS_ANGLE     = 0.2;   // 移动轴方向夹角 < 0.2° → PASS

// 标定图像生成参数
constexpr int NUM_CAMERA_IMAGES    = 24;    // 相机标定图像数
constexpr int NUM_LP_IMAGES        = 8;     // 光平面标定图像数
constexpr int NUM_MA_STEPS         = 5;     // 移动轴标定位置数
constexpr double MA_STEP_MM        = 2.0;   // 移动轴相邻位置间距 (mm)

// ============================================================
// 辅助函数
// ============================================================

/** 输出判定字符串 */
const char* passFail(bool pass) {
    return pass ? "PASS" : "FAIL";
}

/** 计算两个单位向量的夹角（度） */
static double angleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    double d = std::abs(a.normalized().dot(b.normalized()));
    if (d > 1.0) d = 1.0;
    return std::acos(d) * 180.0 / M_PI;
}

// ============================================================
// 主函数
// ============================================================
static bool isSmokeMode(int argc, char* argv[]) {
    return argc > 1 && std::string(argv[1]) == "--smoke";
}

int main(int argc, char* argv[]) {
    if (isSmokeMode(argc, argv)) {
        lsc::sim::SimulatedScanner scanner;
        const auto& k = scanner.getIntrinsics();
        const bool ok = (k.fx > 0.0 && k.fy > 0.0 && k.imgWidth > 0 && k.imgHeight > 0);
        std::cout << "validate_calibration smoke: " << (ok ? "PASS" : "FAIL") << "\n";
        return ok ? 0 : 1;
    }

    std::cout << "========================================\n";
    std::cout << "  标定精度验证报告\n";
    std::cout << "========================================\n\n";

    // --------------------------------------------------
    // 1. 创建模拟扫描器，获取所有 ground truth
    // --------------------------------------------------
    std::cout << "[初始化] 创建 SimulatedScanner，获取 ground truth...\n";

    lsc::sim::SimulatedScanner::Config cfg;
    // 使用默认配置即可：1920×1080, 5.5μm, 16mm 焦距
    // GT 光平面: 0.9063X + 0Y + 0.4226Z - 139.9 = 0
    // GT 移动轴: (0, 1, 0)

    lsc::sim::SimulatedScanner scanner(cfg);

    // 获取 ground truth 参数
    const lsc::CameraIntrinsics& K_gt   = scanner.getIntrinsics();
    const lsc::Plane&            P_gt   = cfg.lightPlane;
    const Eigen::Vector3d&       Ax_gt  = cfg.motionAxis;

    std::cout << "  GT 焦距: fx=" << K_gt.fx << ", fy=" << K_gt.fy << "\n";
    std::cout << "  GT 主点: cx=" << K_gt.cx << ", cy=" << K_gt.cy << "\n";
    std::cout << "  GT 光平面: (" << P_gt.A << ", " << P_gt.B << ", "
              << P_gt.C << ", " << P_gt.D << ")\n";
    std::cout << "  GT 移动轴: (" << Ax_gt.x() << ", " << Ax_gt.y()
              << ", " << Ax_gt.z() << ")\n\n";

    // 用于跟踪综合判定
    bool allPassed = true;

    // --------------------------------------------------
    // [1/3] 相机标定验证
    // --------------------------------------------------
    std::cout << "[1/3] 相机标定\n";
    std::cout << "  生成 " << NUM_CAMERA_IMAGES << " 张棋盘格图像...\n";

    std::vector<cv::Mat> calibImages = scanner.generateCalibImages(NUM_CAMERA_IMAGES);

    lsc::CameraCalibrator camCalib;
    lsc::CameraIntrinsics K_est;
    double rmsCamera = 0.0;

    std::cout << "  运行标定...\n";
    bool camOK = camCalib.calibrateFromImages(
        calibImages,
        cv::Size(cfg.chessboardCols, cfg.chessboardRows),
        cfg.squareSize,
        K_est, rmsCamera);

    if (!camOK) {
        std::cerr << "  [错误] 相机标定失败！\n";
        allPassed = false;
    } else {
        // 逐项对比
        double err_fx = std::abs(K_est.fx - K_gt.fx) / K_gt.fx * 100.0;
        double err_fy = std::abs(K_est.fy - K_gt.fy) / K_gt.fy * 100.0;
        double err_cx = std::abs(K_est.cx - K_gt.cx);
        double err_cy = std::abs(K_est.cy - K_gt.cy);
        double err_k1 = (K_gt.k1 != 0.0) ? std::abs(K_est.k1 - K_gt.k1) / std::abs(K_gt.k1) * 100.0 : 0.0;
        double err_k2 = (K_gt.k2 != 0.0) ? std::abs(K_est.k2 - K_gt.k2) / std::abs(K_gt.k2) * 100.0 : 0.0;

        bool p_fx = (err_fx < TH_FXFY_PCT);
        bool p_fy = (err_fy < TH_FXFY_PCT);
        bool p_cx = (err_cx < TH_CXCY_PX);
        bool p_cy = (err_cy < TH_CXCY_PX);
        // k1/k2 畸变系数不设硬阈值，仅报告

        if (!p_fx || !p_fy || !p_cx || !p_cy) allPassed = false;

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "    参数        Ground Truth      估算值           误差        判定\n";
        std::cout << "    fx         " << std::setw(12) << K_gt.fx
                  << "      " << std::setw(12) << K_est.fx
                  << "       " << std::setw(6) << err_fx << "%"
                  << "     " << passFail(p_fx) << "\n";
        std::cout << "    fy         " << std::setw(12) << K_gt.fy
                  << "      " << std::setw(12) << K_est.fy
                  << "       " << std::setw(6) << err_fy << "%"
                  << "     " << passFail(p_fy) << "\n";
        std::cout << "    cx         " << std::setw(12) << K_gt.cx
                  << "      " << std::setw(12) << K_est.cx
                  << "       " << std::setw(6) << err_cx << "px"
                  << "    " << passFail(p_cx) << "\n";
        std::cout << "    cy         " << std::setw(12) << K_gt.cy
                  << "      " << std::setw(12) << K_est.cy
                  << "       " << std::setw(6) << err_cy << "px"
                  << "    " << passFail(p_cy) << "\n";
        std::cout << "    k1         " << std::setw(12) << K_gt.k1
                  << "      " << std::setw(12) << K_est.k1
                  << "       " << std::setw(6) << err_k1 << "%"
                  << "     --\n";
        std::cout << "    k2         " << std::setw(12) << K_gt.k2
                  << "      " << std::setw(12) << K_est.k2
                  << "       " << std::setw(6) << err_k2 << "%"
                  << "     --\n";
        std::cout << "    重投影误差: " << rmsCamera << " px\n";
    }
    std::cout << "\n";

    // --------------------------------------------------
    // [2/3] 光平面标定验证
    // --------------------------------------------------
    std::cout << "[2/3] 光平面标定\n";
    std::cout << "  生成 " << NUM_LP_IMAGES << " 张激光+棋盘格图像...\n";

    std::vector<cv::Mat> lpImages = scanner.generateLightPlaneCalibImages(NUM_LP_IMAGES);

    lsc::LightPlaneCalibrator lpCalib;
    lsc::Plane P_est;
    double rmsPlane = 0.0;

    std::cout << "  运行光平面标定...\n";
    bool lpOK = lpCalib.calibrate(
        lpImages,
        K_gt,  // 使用 GT 内参进行光平面标定（验证独立光平面精度）
        cv::Size(cfg.chessboardCols, cfg.chessboardRows),
        cfg.squareSize,
        P_est, rmsPlane,
        lsc::LaserMethod::GRAY_CENTROID, 220.0, 1.5);

    if (!lpOK) {
        std::cerr << "  [错误] 光平面标定失败！\n";
        allPassed = false;
    } else {
        // 先归一化光平面法向量再比较
        Eigen::Vector3d n_gt = P_gt.normal();
        Eigen::Vector3d n_est = P_est.normal();

        double angleDeg = angleBetween(n_gt, n_est);
        double dErr = std::abs(P_est.D - P_gt.D);

        bool p_normal = (angleDeg < TH_NORMAL_ANGLE);
        bool p_d = (dErr < TH_D_MM);

        if (!p_normal || !p_d) allPassed = false;

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "    参数         Ground Truth      估算值           误差        判定\n";
        std::cout << "    法向量夹角:  --                --                "
                  << std::setw(6) << angleDeg << "°"
                  << "     " << passFail(p_normal) << "\n";
        std::cout << "    D 值:        " << std::setw(12) << P_gt.D
                  << "      " << std::setw(12) << P_est.D
                  << "       " << std::setw(6) << dErr << "mm"
                  << "    " << passFail(p_d) << "\n";
        std::cout << "    拟合RMS:     --                "
                  << std::setw(12) << rmsPlane << " mm\n";
    }
    std::cout << "\n";

    // --------------------------------------------------
    // [3/3] 移动轴标定验证
    // --------------------------------------------------
    std::cout << "[3/3] 移动轴标定\n";
    std::cout << "  生成 " << NUM_MA_STEPS << " 张移动轴标定图像...\n";

    std::vector<cv::Mat> maImages = scanner.generateMotionAxisCalibImages(NUM_MA_STEPS);

    // 提供每张图像对应的移动轴位置（假设等间距平移）
    std::vector<double> maPositions(NUM_MA_STEPS);
    for (int i = 0; i < NUM_MA_STEPS; ++i) {
        maPositions[i] = static_cast<double>(i) * MA_STEP_MM;
    }

    lsc::MotionAxisCalibrator maCalib;
    Eigen::Vector3d Ax_est;
    double rmsAxis = 0.0;

    std::cout << "  运行动轴标定...\n";
    bool maOK = maCalib.calibrate(
        maImages,
        K_gt,  // 使用 GT 内参进行移动轴标定
        cv::Size(cfg.chessboardCols, cfg.chessboardRows),
        cfg.squareSize,
        maPositions,
        Ax_est, rmsAxis);

    if (!maOK) {
        std::cerr << "  [错误] 移动轴标定失败！\n";
        allPassed = false;
    } else {
        double axisAngleDeg = angleBetween(Ax_gt, Ax_est);

        bool p_axis = (axisAngleDeg < TH_AXIS_ANGLE);
        if (!p_axis) allPassed = false;

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "    参数          Ground Truth      估算值           误差        判定\n";
        std::cout << "    方向向量:     ("
                  << Ax_gt.x() << ", " << Ax_gt.y() << ", " << Ax_gt.z()
                  << ")      (" << Ax_est.x() << ", " << Ax_est.y() << ", "
                  << Ax_est.z() << ")\n";
        std::cout << "    方向夹角:     --                --                "
                  << std::setw(6) << axisAngleDeg << "°"
                  << "     " << passFail(p_axis) << "\n";
        std::cout << "    拟合RMS:      --                "
                  << std::setw(12) << rmsAxis << " mm\n";
    }
    std::cout << "\n";

    // --------------------------------------------------
    // 综合判定
    // --------------------------------------------------
    std::cout << "========================================\n";
    std::cout << "  综合判定: " << passFail(allPassed) << "\n";
    std::cout << "========================================\n";

    return allPassed ? 0 : 1;
}
