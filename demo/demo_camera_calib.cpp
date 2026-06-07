/**
 * @file    demo_camera_calib.cpp
 * @brief   相机标定演示：生成棋盘格图像 → 标定 → 输出内参 + 每张图的重投影误差
 *
 * 使用 SimulatedScanner 生成棋盘格图像，然后调用 CameraCalibrator 进行标定。
 * 如果有 GUI 环境，会显示角点检测结果（否则跳过）。
 */

#include <iostream>
#include <iomanip>
#include <vector>

#include "lsc/core/types.h"
#include "lsc/sim/simulated_scanner.h"
#include "lsc/calib/camera_calib.h"

int main() {
    std::cout << "========================================\n";
    std::cout << "  相机标定演示\n";
    std::cout << "========================================\n\n";

    // --------------------------------------------------
    // 1. 创建模拟扫描器，获取 GT 内参
    // --------------------------------------------------
    std::cout << "[1/3] 创建 SimulatedScanner...\n";

    lsc::sim::SimulatedScanner::Config cfg;
    lsc::sim::SimulatedScanner scanner(cfg);

    const lsc::CameraIntrinsics& K_gt = scanner.getIntrinsics();

    std::cout << "  虚拟相机参数:\n";
    std::cout << "    分辨率: " << K_gt.imgWidth << "x" << K_gt.imgHeight << "\n";
    std::cout << "    焦距: fx=" << std::fixed << std::setprecision(2)
              << K_gt.fx << ", fy=" << K_gt.fy << " px\n";
    std::cout << "    主点: cx=" << K_gt.cx << ", cy=" << K_gt.cy << " px\n";
    std::cout << "    畸变: k1=" << K_gt.k1 << ", k2=" << K_gt.k2 << "\n\n";

    // --------------------------------------------------
    // 2. 生成棋盘格图像并标定
    // --------------------------------------------------
    const int NUM_IMAGES = 24;
    std::cout << "[2/3] 生成 " << NUM_IMAGES << " 张棋盘格图像并标定...\n";

    std::vector<cv::Mat> images = scanner.generateCalibImages(NUM_IMAGES);
    std::cout << "  已生成 " << images.size() << " 张图像\n";

    lsc::CameraCalibrator calibrator;
    lsc::CameraIntrinsics K_est;
    double rmsError = 0.0;

    std::cout << "  运行张正友标定...\n";
    bool ok = calibrator.calibrateFromImages(
        images,
        cv::Size(cfg.chessboardCols, cfg.chessboardRows),
        cfg.squareSize,
        K_est, rmsError);

    if (!ok) {
        std::cerr << "[错误] 标定失败！请检查棋盘格参数。\n";
        return 1;
    }

    std::cout << "  标定成功！总重投影误差: " << rmsError << " px\n\n";

    // --------------------------------------------------
    // 3. 输出标定结果
    // --------------------------------------------------
    std::cout << "[3/3] 标定结果\n\n";

    std::cout << "  ------- 相机内参对比 -------\n";
    std::cout << "  参数   |  Ground Truth  |   估算值    |  误差\n";
    std::cout << "  ----------------------------------------------\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  fx     |  " << std::setw(11) << K_gt.fx
              << "  |  " << std::setw(10) << K_est.fx
              << "  |  " << std::abs(K_est.fx - K_gt.fx) << " px\n";
    std::cout << "  fy     |  " << std::setw(11) << K_gt.fy
              << "  |  " << std::setw(10) << K_est.fy
              << "  |  " << std::abs(K_est.fy - K_gt.fy) << " px\n";
    std::cout << "  cx     |  " << std::setw(11) << K_gt.cx
              << "  |  " << std::setw(10) << K_est.cx
              << "  |  " << std::abs(K_est.cx - K_gt.cx) << " px\n";
    std::cout << "  cy     |  " << std::setw(11) << K_gt.cy
              << "  |  " << std::setw(10) << K_est.cy
              << "  |  " << std::abs(K_est.cy - K_gt.cy) << " px\n";
    std::cout << "  k1     |  " << std::setw(11) << K_gt.k1
              << "  |  " << std::setw(10) << K_est.k1
              << "  |  " << std::abs(K_est.k1 - K_gt.k1) << "\n";
    std::cout << "  k2     |  " << std::setw(11) << K_gt.k2
              << "  |  " << std::setw(10) << K_est.k2
              << "  |  " << std::abs(K_est.k2 - K_gt.k2) << "\n\n";

    // 逐张图像重投影误差
    std::vector<double> perView = calibrator.perViewErrors();
    if (!perView.empty()) {
        std::cout << "  ------- 逐张重投影误差 -------\n";
        for (size_t i = 0; i < perView.size(); ++i) {
            std::cout << "  图像[" << std::setw(2) << (i + 1) << "]: "
                      << std::setprecision(4) << perView[i] << " px\n";
        }
    }

    // 保存标定结果
    std::string outPath = "output/camera_params.yaml";
    if (calibrator.save(outPath, K_est)) {
        std::cout << "\n  标定结果已保存到: " << outPath << "\n";
    }

    // 尝试显示角点检测（无 GUI 则跳过）
    std::cout << "\n  [提示] 如需可视化角点检测结果，请在有 GUI 环境下运行。";
    std::cout << "\n         当前将跳过图像显示。\n";

    std::cout << "\n========================================\n";
    std::cout << "  相机标定演示完成\n";
    std::cout << "========================================\n";

    return 0;
}
