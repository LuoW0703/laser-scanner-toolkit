/**
 * @file    validate_repeatability.cpp
 * @brief   重复性验证：对同一阶梯块重复扫描 30 次，统计台阶高度测量的重复性
 *
 * 流程：
 *   1. 创建 30 个不同噪声种子的 SimulatedScanner（注：当前 API 固定默认种子，
 *      此处通过微调 config 参数模拟不同噪声实现，实际工程中应支持 setSeed 接口）
 *   2. 每次重建后测量台阶高度
 *   3. 统计均值、标准差、极差
 *
 * 注意：由于当前 SimulatedScanner 的 mt19937 使用固定默认种子 5489，
 * 无法直接改变随机种子。本验证通过微调 sensorNoise 来模拟不同噪声环境，
 * 获取统计上的变化。在实际工程中建议为 SimulatedScanner 添加 setSeed() 方法。
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <numeric>
#include <random>
#include <string>

#include "lsc/core/types.h"
#include "lsc/sim/simulated_scanner.h"
#include "lsc/recon/reconstructor.h"

// ============================================================
// 验证参数
// ============================================================
constexpr int    NUM_REPEATS      = 30;    // 重复扫描次数
constexpr double TH_STDDEV_MM     = 0.05;  // 标准差 < 0.05mm → PASS
constexpr double TOLERANCE_MM     = 0.2;   // Cpk 容差 ±0.2mm

// 阶梯块参数
constexpr double BLOCK_WIDTH      = 60.0;
constexpr double BLOCK_DEPTH      = 40.0;
constexpr double STEP1_HEIGHT     = 10.0;
constexpr double STEP2_HEIGHT     = 20.0;

/**
 * 一次完整的扫描+重建+高度测量流程。
 * @return {台阶1高度, 台阶2高度}
 */
static std::pair<double, double> runOneScan(double noiseSigma) {
    lsc::sim::SimulatedScanner::Config cfg;
    cfg.sensorNoise = noiseSigma;

    lsc::sim::SimulatedScanner scanner(cfg);

    const lsc::CameraIntrinsics& K_gt  = scanner.getIntrinsics();
    const lsc::Plane&            P_gt  = cfg.lightPlane;
    const Eigen::Vector3d&       Ax_gt = cfg.motionAxis;

    lsc::Reconstructor recon(K_gt, P_gt, Ax_gt);

    lsc::sim::StepBlock block;
    block.width       = BLOCK_WIDTH;
    block.depth       = BLOCK_DEPTH;
    block.stepHeights = {STEP1_HEIGHT, STEP2_HEIGHT};

    auto [scanImages, gtCloud] = scanner.generateScan(block);

    // 提取激光线 + 重建
    lsc::LaserLineExtractor extractor;
    std::vector<lsc::ScanLine> scanLines;
    for (size_t i = 0; i < scanImages.size(); ++i) {
        auto pts = extractor.extract(scanImages[i],
                                     lsc::LaserMethod::GRAY_CENTROID, 220.0, 1.5);
        lsc::ScanLine line;
        line.laserPoints    = pts;
        line.motionPosition = scanner.scanPosition(i);
        line.hasMotionPosition = true;
        scanLines.push_back(line);
    }

    lsc::PointCloud reconCloud = recon.reconstruct(scanLines, cfg.defaultStep);

    // 提取台阶高度（简化方法：X 分区 + 找 Z 阶梯）
    if (reconCloud.empty()) return {0.0, 0.0};

    // 将点云按 X 分成 3 个区域（假设 3 个等宽台阶面），计算每个区域的平均 Z
    double xMin = std::numeric_limits<double>::max();
    double xMax = std::numeric_limits<double>::lowest();
    for (const auto& p : reconCloud) {
        if (p.x() < xMin) xMin = p.x();
        if (p.x() > xMax) xMax = p.x();
    }

    double segW = (xMax - xMin) / 3.0;
    std::vector<std::vector<double>> zoneZ(3);
    for (const auto& p : reconCloud) {
        int zi = static_cast<int>((p.x() - xMin) / segW);
        if (zi < 0) zi = 0;
        if (zi > 2) zi = 2;
        zoneZ[zi].push_back(p.z());
    }

    std::vector<double> zoneMeanZ(3, 0.0);
    for (int i = 0; i < 3; ++i) {
        if (!zoneZ[i].empty()) {
            zoneMeanZ[i] = std::accumulate(zoneZ[i].begin(), zoneZ[i].end(), 0.0)
                           / static_cast<double>(zoneZ[i].size());
        }
    }

    double h1 = zoneMeanZ[1] - zoneMeanZ[0];
    double h2 = zoneMeanZ[2] - zoneMeanZ[0];
    return {h1, h2};
}

/**
 * 计算 Cpk (Process Capability Index)
 * Cpk = min(USL - mu, mu - LSL) / (3 * sigma)
 */
static double computeCpk(double mean, double stddev, double nominal, double tolerance) {
    if (stddev < 1e-12) return 99.9;  // 几乎无变异
    double usl = nominal + tolerance;
    double lsl = nominal - tolerance;
    return std::min(usl - mean, mean - lsl) / (3.0 * stddev);
}

// ============================================================
// 主函数
// ============================================================
static bool isSmokeMode(int argc, char* argv[]) {
    return argc > 1 && std::string(argv[1]) == "--smoke";
}

int main(int argc, char* argv[]) {
    if (isSmokeMode(argc, argv)) {
        auto [h1, h2] = runOneScan(1.0);
        const bool ok = std::isfinite(h1) && std::isfinite(h2);
        std::cout << "validate_repeatability smoke: " << (ok ? "PASS" : "FAIL")
                  << " (h1=" << h1 << ", h2=" << h2 << ")\n";
        return ok ? 0 : 1;
    }

    std::cout << "========================================\n";
    std::cout << "  重复性验证报告 (" << NUM_REPEATS << "次重复扫描)\n";
    std::cout << "========================================\n\n";

    std::vector<double> h1Results, h2Results;
    h1Results.reserve(NUM_REPEATS);
    h2Results.reserve(NUM_REPEATS);

    std::cout << "[扫描] 执行 " << NUM_REPEATS << " 次重复扫描...\n";

    // 生成随机种子列表：通过略微变动噪声等级模拟真实世界的波动
    // 基准噪声 1.0，加上 [-0.2, 0.2] 范围内的随机扰动
    std::mt19937 rng(42);  // 使用固定种子以保证可复现
    std::uniform_real_distribution<double> noiseJitter(-0.2, 0.2);

    for (int i = 0; i < NUM_REPEATS; ++i) {
        double noiseVal = 1.0 + noiseJitter(rng);
        auto [h1, h2] = runOneScan(noiseVal);
        h1Results.push_back(h1);
        h2Results.push_back(h2);

        if ((i + 1) % 5 == 0 || i + 1 == NUM_REPEATS) {
            std::cout << "  ... " << (i + 1) << "/" << NUM_REPEATS
                      << " 次完成 (最近 h1=" << std::fixed << std::setprecision(3)
                      << h1 << ", h2=" << h2 << ")\n";
        }
    }
    std::cout << "\n";

    // --------------------------------------------------
    // 统计分析
    // --------------------------------------------------
    auto compute = [](const std::vector<double>& v) {
        if (v.empty()) return std::make_tuple(0.0, 0.0, 0.0);
        double sum = std::accumulate(v.begin(), v.end(), 0.0);
        double mean = sum / v.size();

        double var = 0.0;
        for (double x : v) var += (x - mean) * (x - mean);
        double stddev = std::sqrt(var / v.size());

        double minVal = *std::min_element(v.begin(), v.end());
        double maxVal = *std::max_element(v.begin(), v.end());
        double range = maxVal - minVal;
        return std::make_tuple(mean, stddev, range);
    };

    auto [mean1, std1, range1] = compute(h1Results);
    auto [mean2, std2, range2] = compute(h2Results);

    double cpk1 = computeCpk(mean1, std1, STEP1_HEIGHT, TOLERANCE_MM);
    double cpk2 = computeCpk(mean2, std2, STEP2_HEIGHT, TOLERANCE_MM);

    // 判定
    bool pass1 = (std1 < TH_STDDEV_MM);
    bool pass2 = (std2 < TH_STDDEV_MM);
    bool allPassed = pass1 && pass2;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "台阶1高度 (真实值 " << STEP1_HEIGHT << " mm):\n";
    std::cout << "  均值: " << mean1 << " mm\n";
    std::cout << "  标准差: " << std1 << " mm  (通过标准: < " << TH_STDDEV_MM
              << "mm)  " << (pass1 ? "PASS" : "FAIL") << "\n";
    std::cout << "  极差: " << range1 << " mm\n";
    std::cout << "  Cpk: " << std::setprecision(2) << cpk1
              << "  (容差 +/-" << TOLERANCE_MM << "mm)\n\n";

    std::cout << "台阶2高度 (真实值 " << STEP2_HEIGHT << " mm):\n";
    std::cout << "  均值: " << mean2 << " mm\n";
    std::cout << "  标准差: " << std2 << " mm  (通过标准: < " << TH_STDDEV_MM
              << "mm)  " << (pass2 ? "PASS" : "FAIL") << "\n";
    std::cout << "  极差: " << range2 << " mm\n";
    std::cout << "  Cpk: " << std::setprecision(2) << cpk2
              << "  (容差 +/-" << TOLERANCE_MM << "mm)\n\n";

    // 说明当前 API 限制
    std::cout << "[说明] 当前 SimulatedScanner 使用固定默认随机种子，\n";
    std::cout << "       本验证通过微调噪声等级模拟不同测量环境。\n";
    std::cout << "       建议为 SimulatedScanner 添加 setSeed() 方法以支持真正的重复性测试。\n\n";

    // --------------------------------------------------
    // 综合判定
    // --------------------------------------------------
    std::cout << "========================================\n";
    std::cout << "  综合判定: " << (allPassed ? "PASS" : "FAIL") << "\n";
    std::cout << "========================================\n";

    return allPassed ? 0 : 1;
}
