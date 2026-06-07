/**
 * @file    validate_robustness.cpp
 * @brief   噪声鲁棒性验证：测试不同传感器噪声等级下的重建精度退化曲线
 *
 * 流程：
 *   1. 在 5 个噪声等级下 (sensorNoise = 0.5, 1.0, 2.0, 3.0, 5.0) 各扫描 10 次
 *   2. 记录每个噪声等级下的 RMSE 和体积误差
 *   3. 输出退化曲线数据并给出结论
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <numeric>
#include <string>

#include "lsc/core/types.h"
#include "lsc/sim/simulated_scanner.h"
#include "lsc/recon/reconstructor.h"
#include "lsc/proc/measurement.h"

// ============================================================
// 验证参数
// ============================================================
constexpr double TH_RMSE_ACCEPTABLE = 0.3;   // RMSE < 0.3mm 视为可接受
constexpr int    SCANS_PER_LEVEL    = 10;    // 每个噪声等级扫描次数

// 阶梯块参数
constexpr double BLOCK_WIDTH      = 60.0;
constexpr double BLOCK_DEPTH      = 40.0;
constexpr double STEP1_HEIGHT     = 10.0;
constexpr double STEP2_HEIGHT     = 20.0;

// 真实体积
constexpr double SEG_W = BLOCK_WIDTH / 3.0;
constexpr double GT_VOL = SEG_W * BLOCK_DEPTH * 0.0
                         + SEG_W * BLOCK_DEPTH * STEP1_HEIGHT
                         + SEG_W * BLOCK_DEPTH * STEP2_HEIGHT;

/**
 * 计算两个点云之间的 RMSE（单向，从 pc 到 ref）
 */
static double computeRMSE(const lsc::PointCloud& pc, const lsc::PointCloud& ref) {
    if (pc.empty() || ref.empty()) return std::numeric_limits<double>::quiet_NaN();
    double sumSq = 0.0;
    for (const auto& p : pc) {
        double minD2 = std::numeric_limits<double>::max();
        for (const auto& q : ref) {
            double dx = p.x() - q.x(), dy = p.y() - q.y(), dz = p.z() - q.z();
            double d2 = dx*dx + dy*dy + dz*dz;
            if (d2 < minD2) minD2 = d2;
        }
        sumSq += minD2;
    }
    return std::sqrt(sumSq / pc.size());
}

/**
 * 一次扫描+重建流程。
 * @return {RMSE, 体积测量值, 重建点云点数}
 */
struct ScanResult {
    double rmse = 0.0;
    double volume = 0.0;
    int numPoints = 0;
};

static ScanResult runOneScan(double noiseSigma, const lsc::PointCloud& gtSampled) {
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

    // 提取 + 重建
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

    ScanResult result;
    result.numPoints = static_cast<int>(reconCloud.size());

    if (reconCloud.empty()) {
        result.rmse = std::numeric_limits<double>::quiet_NaN();
        result.volume = 0.0;
        return result;
    }

    // RMSE (使用传入的采样 GT)
    result.rmse = computeRMSE(reconCloud, gtSampled);

    // 体积
    double zMin = std::numeric_limits<double>::max();
    for (const auto& p : reconCloud) {
        if (p.z() < zMin) zMin = p.z();
    }
    Eigen::Vector4d refPlane(0.0, 0.0, 1.0, -zMin);
    result.volume = lsc::PointCloudMeasurer::computeVolume(reconCloud, refPlane, 0.5);

    return result;
}

// ============================================================
// 主函数
// ============================================================
static bool isSmokeMode(int argc, char* argv[]) {
    return argc > 1 && std::string(argv[1]) == "--smoke";
}

int main(int argc, char* argv[]) {
    if (isSmokeMode(argc, argv)) {
        lsc::sim::SimulatedScanner::Config cfg;
        lsc::sim::SimulatedScanner scanner(cfg);
        lsc::sim::StepBlock block;
        block.width = BLOCK_WIDTH;
        block.depth = BLOCK_DEPTH;
        block.stepHeights = {STEP1_HEIGHT, STEP2_HEIGHT};
        auto [scanImages, gtCloud] = scanner.generateScan(block);
        double xMin = std::numeric_limits<double>::max();
        double xMax = std::numeric_limits<double>::lowest();
        double yMin = std::numeric_limits<double>::max();
        double yMax = std::numeric_limits<double>::lowest();
        for (const auto& point : gtCloud) {
            xMin = std::min(xMin, point.x());
            xMax = std::max(xMax, point.x());
            yMin = std::min(yMin, point.y());
            yMax = std::max(yMax, point.y());
        }
        const double xSpan = xMax - xMin;
        const double ySpan = yMax - yMin;
        // GT 必须位于工件自身坐标系。若运动位移被重复累计，
        // 任一平面尺寸都会明显超过模型的 width/depth。
        const bool geometryOk =
            xSpan <= block.width + 1e-6 &&
            ySpan <= block.depth + 1e-6;
        const bool ok =
            !scanImages.empty() && !gtCloud.empty() && geometryOk;
        std::cout << "validate_robustness smoke: " << (ok ? "PASS" : "FAIL") << "\n";
        return ok ? 0 : 1;
    }

    std::cout << "========================================\n";
    std::cout << "  鲁棒性验证报告\n";
    std::cout << "========================================\n\n";

    // 噪声等级列表
    std::vector<double> noiseLevels = {0.5, 1.0, 2.0, 3.0, 5.0};

    // 预先准备一份采样的 GT 点云用于 RMSE 计算
    // 使用默认噪声等级生成
    lsc::sim::SimulatedScanner::Config defaultCfg;
    lsc::sim::SimulatedScanner refScanner(defaultCfg);
    lsc::sim::StepBlock block;
    block.width = BLOCK_WIDTH; block.depth = BLOCK_DEPTH;
    block.stepHeights = {STEP1_HEIGHT, STEP2_HEIGHT};
    auto [_, gtCloud] = refScanner.generateScan(block);

    // 采样 GT 点云
    lsc::PointCloud gtSampled;
    int gtStep = std::max(1, static_cast<int>(gtCloud.size()) / 5000);
    for (size_t i = 0; i < gtCloud.size(); i += gtStep) {
        gtSampled.push_back(gtCloud[i]);
    }

    // 存储每个噪声等级的汇总结果
    struct LevelResult {
        double noise;
        double meanRMSE;
        double meanVolErr;
    };
    std::vector<LevelResult> results;

    std::cout << "[测试] 逐噪声等级扫描...\n\n";

    for (double noise : noiseLevels) {
        std::cout << "  噪声 sigma=" << std::setprecision(1) << noise << "\n";

        double sumRMSE = 0.0, sumVolErr = 0.0;
        int validScans = 0;

        for (int s = 0; s < SCANS_PER_LEVEL; ++s) {
            ScanResult sr = runOneScan(noise, gtSampled);

            if (!std::isnan(sr.rmse)) {
                sumRMSE += sr.rmse;
                sumVolErr += std::abs(sr.volume - GT_VOL) / GT_VOL * 100.0;
                validScans++;
            }

            if ((s + 1) % 3 == 0 || s + 1 == SCANS_PER_LEVEL) {
                std::cout << "    ... " << (s + 1) << "/" << SCANS_PER_LEVEL
                          << " 次完成\n";
            }
        }

        double meanRMSE = (validScans > 0) ? sumRMSE / validScans : 0.0;
        double meanVolErr = (validScans > 0) ? sumVolErr / validScans : 0.0;

        results.push_back({noise, meanRMSE, meanVolErr});
        std::cout << "    平均 RMSE: " << std::fixed << std::setprecision(3)
                  << meanRMSE << " mm, 平均体积误差: "
                  << std::setprecision(1) << meanVolErr << "%\n\n";
    }

    // --------------------------------------------------
    // 汇总表格
    // --------------------------------------------------
    std::cout << "========================================\n";
    std::cout << "  传感器噪声对精度的影响\n";
    std::cout << "========================================\n";
    std::cout << "  sigma  |  RMSE (mm)  |  体积误差 (%)\n";
    std::cout << "  -------------------------------------\n";

    double maxAcceptableNoise = 0.0;
    for (const auto& r : results) {
        std::cout << "  " << std::setw(5) << std::setprecision(1) << r.noise
                  << "  |  " << std::setw(8) << std::setprecision(3) << r.meanRMSE
                  << "   |  " << std::setw(9) << std::setprecision(1) << r.meanVolErr
                  << "\n";
        if (r.meanRMSE < TH_RMSE_ACCEPTABLE) {
            maxAcceptableNoise = r.noise;
        }
    }

    // 结论
    std::cout << "\n  结论: 在传感器噪声 sigma <= "
              << std::setprecision(1) << maxAcceptableNoise
              << " 时，RMSE < " << TH_RMSE_ACCEPTABLE
              << " mm (精度可接受范围)\n";
    std::cout << "========================================\n";

    // 如果至少最低噪声等级通过，视为整体 PASS
    bool passed = (!results.empty() && results[0].meanRMSE < TH_RMSE_ACCEPTABLE);
    std::cout << "  综合判定: " << (passed ? "PASS" : "FAIL") << "\n";
    std::cout << "========================================\n";

    return passed ? 0 : 1;
}
