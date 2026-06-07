/**
 * @file    demo_light_plane_calib.cpp
 * @brief   光平面标定演示：生成激光+棋盘格叠加图像 → 提取激光线 → 标定光平面
 *
 * 使用 SimulatedScanner 生成光平面标定图像（棋盘格 + 激光线叠加），
 * 提取激光线中心，拟合光平面方程。
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>

#include "lsc/core/types.h"
#include "lsc/sim/simulated_scanner.h"
#include "lsc/calib/light_plane_calib.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main() {
    std::cout << "========================================\n";
    std::cout << "  光平面标定演示\n";
    std::cout << "========================================\n\n";

    // --------------------------------------------------
    // 1. 创建模拟扫描器，获取 GT 参数
    // --------------------------------------------------
    std::cout << "[1/3] 创建 SimulatedScanner...\n";

    lsc::sim::SimulatedScanner::Config cfg;
    lsc::sim::SimulatedScanner scanner(cfg);

    const lsc::CameraIntrinsics& K_gt = scanner.getIntrinsics();
    const lsc::Plane&            P_gt = cfg.lightPlane;

    std::cout << "  相机内参已就绪 (fx=" << std::fixed << std::setprecision(2)
              << K_gt.fx << ", fy=" << K_gt.fy << ")\n";
    std::cout << "  GT 光平面: (" << P_gt.A << ", " << P_gt.B << ", "
              << P_gt.C << ", " << P_gt.D << ")\n\n";

    // --------------------------------------------------
    // 2. 生成光平面标定图像并标定
    // --------------------------------------------------
    const int NUM_IMAGES = 8;
    std::cout << "[2/3] 生成 " << NUM_IMAGES << " 张激光+棋盘格图像...\n";

    std::vector<cv::Mat> images = scanner.generateLightPlaneCalibImages(NUM_IMAGES);
    std::cout << "  已生成 " << images.size() << " 张图像\n";
    std::cout << "  每张图像同时包含棋盘格和激光线叠加\n";

    std::cout << "\n  运行光平面标定...\n";
    std::cout << "  提取方法: 灰度重心法 (GRAY_CENTROID)\n";

    lsc::LightPlaneCalibrator calibrator;
    lsc::Plane P_est;
    double rmsError = 0.0;

    bool ok = calibrator.calibrate(
        images,
        K_gt,  // 使用 GT 内参（实际工程中应使用 CameraCalibrator 标定结果）
        cv::Size(cfg.chessboardCols, cfg.chessboardRows),
        cfg.squareSize,
        P_est, rmsError,
        lsc::LaserMethod::GRAY_CENTROID,
        220.0,  // 仿真棋盘白格低于激光峰值
        1.5     // 高斯 sigma
    );

    if (!ok) {
        std::cerr << "[错误] 光平面标定失败！\n";
        std::cerr << "  请检查：1) 激光线是否可见  2) 棋盘格是否被检测到  3) 激光线提取阈值是否合适\n";
        return 1;
    }

    std::cout << "  标定成功！平面拟合 RMS: " << rmsError << " mm\n\n";

    // --------------------------------------------------
    // 3. 输出标定结果与对比
    // --------------------------------------------------
    std::cout << "[3/3] 标定结果\n\n";

    // 归一化法向量
    Eigen::Vector3d n_gt = P_gt.normal();
    Eigen::Vector3d n_est = P_est.normal();

    // 计算法向量夹角
    double dotVal = std::abs(n_gt.dot(n_est));
    if (dotVal > 1.0) dotVal = 1.0;
    double angleDeg = std::acos(dotVal) * 180.0 / M_PI;

    // D 值误差
    double dErr = std::abs(P_est.D - P_gt.D);

    std::cout << "  -------- 光平面对比 --------\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  参数         |  Ground Truth   |   估算值\n";
    std::cout << "  ---------------------------------------------\n";
    std::cout << "  法向量 A     |  " << std::setw(12) << P_gt.A
              << "  |  " << std::setw(12) << P_est.A << "\n";
    std::cout << "  法向量 B     |  " << std::setw(12) << P_gt.B
              << "  |  " << std::setw(12) << P_est.B << "\n";
    std::cout << "  法向量 C     |  " << std::setw(12) << P_gt.C
              << "  |  " << std::setw(12) << P_est.C << "\n";
    std::cout << "  D 值         |  " << std::setw(12) << P_gt.D
              << "  |  " << std::setw(12) << P_est.D << "\n";
    std::cout << "  ---------------------------------------------\n";
    std::cout << "  法向量夹角: " << std::setprecision(3) << angleDeg << " 度\n";
    std::cout << "  D 值误差:   " << dErr << " mm\n";

    // 判定
    bool pass = (angleDeg < 0.5 && dErr < 1.0);
    std::cout << "\n  判定: " << (pass ? "PASS" : "FAIL")
              << "  (法向量夹角 < 0.5° 且 D 值误差 < 1mm)\n";

    std::cout << "\n========================================\n";
    std::cout << "  光平面标定演示完成\n";
    std::cout << "========================================\n";

    return 0;
}
