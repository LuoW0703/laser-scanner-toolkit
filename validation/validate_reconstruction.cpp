/**
 * @file    validate_reconstruction.cpp
 * @brief   重建精度验证：对比重建点云与 ground truth 模型
 *
 * 流程：
 *   1. 创建 SimulatedScanner，用已知 GT 参数的标定结果
 *   2. 生成阶梯块扫描数据 + ground truth 点云
 *   3. Reconstructor 重建点云
 *   4. 计算重建点云到 ground truth 模型的距离误差 (RMSE)
 *   5. 提取台阶轮廓并测量台阶高度
 *   6. 用 PointCloudMeasurer 测量体积
 *   7. 与真实值对比并输出报告
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
// 判定阈值（写死在代码中）
// ============================================================
constexpr double TH_RMSE_MM       = 0.3;   // 点云 RMSE < 0.3mm → PASS
constexpr double TH_STEP_HEIGHT   = 0.2;   // 台阶高度误差 < 0.2mm → PASS
constexpr double TH_VOLUME_PCT    = 2.0;   // 体积误差 < 2% → PASS

// 阶梯块参数
constexpr double BLOCK_WIDTH      = 60.0;  // mm (沿 X)
constexpr double BLOCK_DEPTH      = 40.0;  // mm (沿 Y)
constexpr double STEP1_GT_HEIGHT  = 10.0;  // mm 台阶1高度
constexpr double STEP2_GT_HEIGHT  = 20.0;  // mm 台阶2高度

// 点云测量参数
constexpr int    X_BINS           = 60;    // X 方向分区数，用于提取台阶轮廓

// ============================================================
// 辅助函数
// ============================================================

/** 输出判定字符串 */
static const char* passFail(bool pass) {
    return pass ? "PASS" : "FAIL";
}

/**
 * 计算点云 pc 中每个点到参考点云 ref 最近点的距离，
 * 返回均方根误差 (RMSE, mm)。
 * 算法：暴力最近邻搜索 O(n*m)，适用于验证场景中规模可控的点云。
 */
static double computeRMSE(const lsc::PointCloud& pc, const lsc::PointCloud& ref) {
    if (pc.empty() || ref.empty()) return std::numeric_limits<double>::quiet_NaN();

    double sumSq = 0.0;
    int progressStep = std::max(1, static_cast<int>(pc.size()) / 10);
    for (size_t i = 0; i < pc.size(); ++i) {
        const auto& p = pc[i];
        double minDistSq = std::numeric_limits<double>::max();
        for (const auto& q : ref) {
            double dx = p.x() - q.x();
            double dy = p.y() - q.y();
            double dz = p.z() - q.z();
            double d2 = dx*dx + dy*dy + dz*dz;
            if (d2 < minDistSq) minDistSq = d2;
        }
        sumSq += minDistSq;

        if ((i + 1) % progressStep == 0) {
            std::cout << "  ... RMSE 计算进度 " << (i + 1) << "/" << pc.size() << "\n";
        }
    }
    return std::sqrt(sumSq / static_cast<double>(pc.size()));
}

/**
 * 将点云按 X 坐标分 bin，每个 bin 内取 Z 坐标上分位数作为顶面高度。
 * 用于从重建点云中提取阶梯块的台阶轮廓。
 *
 * @param cloud   输入点云
 * @param nBins   X 方向分区数
 * @param outX    输出每个 bin 的 X 中心坐标
 * @param outZ    输出每个 bin 的 Z 顶面高度
 */
static void extractStepProfile(const lsc::PointCloud& cloud, int nBins,
                               std::vector<double>& outX,
                               std::vector<double>& outZ) {
    if (cloud.empty()) return;

    double xMin = std::numeric_limits<double>::max();
    double xMax = std::numeric_limits<double>::lowest();
    for (const auto& p : cloud) {
        if (p.x() < xMin) xMin = p.x();
        if (p.x() > xMax) xMax = p.x();
    }

    double binWidth = (xMax - xMin) / static_cast<double>(nBins);
    if (binWidth <= 0.0) return;

    // 每个 bin 收集 Z 值
    std::vector<std::vector<double>> binZ(nBins);
    for (const auto& p : cloud) {
        int idx = static_cast<int>((p.x() - xMin) / binWidth);
        if (idx < 0) idx = 0;
        if (idx >= nBins) idx = nBins - 1;
        binZ[idx].push_back(p.z());
    }

    outX.resize(nBins);
    outZ.resize(nBins);
    for (int i = 0; i < nBins; ++i) {
        outX[i] = xMin + (i + 0.5) * binWidth;
        if (binZ[i].size() < 3) {
            outZ[i] = 0.0;
        } else {
            // 取上分位数 (90%) 作为表面高度，避免噪声点干扰
            std::sort(binZ[i].begin(), binZ[i].end());
            size_t idx90 = static_cast<size_t>(binZ[i].size() * 0.9);
            if (idx90 >= binZ[i].size()) idx90 = binZ[i].size() - 1;
            outZ[i] = binZ[i][idx90];
        }
    }
}

/**
 * 从台阶轮廓 Z 值序列中识别各台阶面的高度。
 * 通过聚类（相邻 Z 差值超过 clusterTol 即分群）来检测台阶面。
 * 返回各台阶面估计高度（从低到高排序）。
 */
static std::vector<double> detectStepLevels(const std::vector<double>& profileZ,
                                             double clusterTol = 5.0) {
    // 收集有效 Z 值（过滤零值和异常值）
    std::vector<double> zVals;
    for (double z : profileZ) {
        if (z > 0.0) zVals.push_back(z);
    }
    if (zVals.empty()) return {};

    std::sort(zVals.begin(), zVals.end());

    // 聚类：相邻 Z 差值 > clusterTol 则为不同台阶面
    std::vector<std::vector<double>> clusters;
    clusters.push_back({zVals[0]});
    for (size_t i = 1; i < zVals.size(); ++i) {
        if (zVals[i] - zVals[i - 1] > clusterTol) {
            clusters.push_back({zVals[i]});
        } else {
            clusters.back().push_back(zVals[i]);
        }
    }

    // 每个聚类的均值作为台阶面高度（要求至少 5 个 bin 才算有效）
    std::vector<double> levels;
    for (const auto& c : clusters) {
        if (c.size() >= 5) {
            double mean = std::accumulate(c.begin(), c.end(), 0.0) / static_cast<double>(c.size());
            levels.push_back(mean);
        }
    }
    return levels;
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
        lsc::sim::StepBlock block;
        block.width = BLOCK_WIDTH;
        block.depth = BLOCK_DEPTH;
        block.stepHeights = {STEP1_GT_HEIGHT, STEP2_GT_HEIGHT};
        auto [scanImages, gtCloud] = scanner.generateScan(block);
        const bool ok = !scanImages.empty() && !gtCloud.empty();
        std::cout << "validate_reconstruction smoke: " << (ok ? "PASS" : "FAIL") << "\n";
        return ok ? 0 : 1;
    }

    std::cout << "========================================\n";
    std::cout << "  重建精度验证报告\n";
    std::cout << "========================================\n\n";

    // --------------------------------------------------
    // 1. 创建模拟扫描器并配置阶梯块
    // --------------------------------------------------
    std::cout << "[初始化] 创建 SimulatedScanner...\n";

    lsc::sim::SimulatedScanner::Config cfg;
    // 使用 GT 标定参数直接构造 Reconstructor，排除标定误差
    // 这样验证的是重建算法本身的精度

    lsc::sim::SimulatedScanner scanner(cfg);

    const lsc::CameraIntrinsics& K_gt  = scanner.getIntrinsics();
    const lsc::Plane&            P_gt  = cfg.lightPlane;
    const Eigen::Vector3d&       Ax_gt = cfg.motionAxis;

    lsc::Reconstructor recon(K_gt, P_gt, Ax_gt);

    // 配置阶梯块
    lsc::sim::StepBlock block;
    block.width       = BLOCK_WIDTH;
    block.depth       = BLOCK_DEPTH;
    block.stepHeights = {STEP1_GT_HEIGHT, STEP2_GT_HEIGHT};

    int numLines = static_cast<int>(cfg.travelRange / cfg.defaultStep) + 1;
    std::cout << "  被测物体: 阶梯块 (" << BLOCK_WIDTH << "x" << BLOCK_DEPTH
              << "mm, 台阶 " << STEP1_GT_HEIGHT << "mm, " << STEP2_GT_HEIGHT << "mm)\n";
    std::cout << "  扫描线数: " << numLines << " (步长 " << cfg.defaultStep
              << "mm, 行程 " << cfg.travelRange << "mm)\n\n";

    // --------------------------------------------------
    // 2. 生成扫描数据 + GT 点云
    // --------------------------------------------------
    std::cout << "[扫描生成] 生成阶梯块模拟扫描数据...\n";

    auto [scanImages, gtCloud] = scanner.generateScan(block);

    std::cout << "  扫描图像: " << scanImages.size() << " 帧\n";
    std::cout << "  GT 点云点数: " << gtCloud.size() << "\n\n";

    // --------------------------------------------------
    // 3. 激光线提取 + 构建 ScanLine + 重建
    // --------------------------------------------------
    std::cout << "[重建] 从扫描图像重建 3D 点云...\n";

    lsc::LaserLineExtractor extractor;
    std::vector<lsc::ScanLine> scanLines;
    scanLines.reserve(scanImages.size());

    int progressStep = std::max(1, static_cast<int>(scanImages.size()) / 10);
    for (size_t i = 0; i < scanImages.size(); ++i) {
        auto pts = extractor.extract(scanImages[i],
                                     lsc::LaserMethod::GRAY_CENTROID, 220.0, 1.5);

        lsc::ScanLine line;
        line.laserPoints    = pts;
        line.motionPosition = scanner.scanPosition(i);
        line.hasMotionPosition = true;
        scanLines.push_back(line);

        if ((i + 1) % progressStep == 0 || i + 1 == scanImages.size()) {
            std::cout << "  ... 已处理 " << (i + 1) << "/" << scanImages.size() << " 帧\n";
        }
    }

    lsc::PointCloud reconCloud = recon.reconstruct(scanLines, cfg.defaultStep);
    std::cout << "  重建点云点数: " << reconCloud.size() << "\n\n";

    if (reconCloud.empty()) {
        std::cerr << "[错误] 重建点云为空！请检查激光线提取参数。\n";
        return 1;
    }

    // --------------------------------------------------
    // 4. 计算 RMSE
    // --------------------------------------------------
    std::cout << "[精度验证] 计算重建点云到 GT 模型的 RMSE...\n";

    // 对 GT 点云做均匀采样以加速最近邻搜索
    lsc::PointCloud gtSampled;
    int gtSampleStep = std::max(1, static_cast<int>(gtCloud.size()) / 10000);
    for (size_t i = 0; i < gtCloud.size(); i += gtSampleStep) {
        gtSampled.push_back(gtCloud[i]);
    }
    std::cout << "  GT 采样点数: " << gtSampled.size() << " (用于加速计算)\n";

    double rmse = computeRMSE(reconCloud, gtSampled);

    bool allPassed = true;
    bool pRMSE = (!std::isnan(rmse) && rmse < TH_RMSE_MM);
    if (!pRMSE) allPassed = false;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n  重建点云点数: " << reconCloud.size() << "\n";
    std::cout << "  GT 点云点数:   " << gtCloud.size() << "\n";
    std::cout << "  点云到 GT 模型的 RMSE: " << rmse << " mm  "
              << passFail(pRMSE) << "\n\n";

    // --------------------------------------------------
    // 5. 台阶高度测量
    // --------------------------------------------------
    std::cout << "[尺寸测量] 从重建点云提取台阶高度...\n";

    std::vector<double> profileX, profileZ;
    extractStepProfile(reconCloud, X_BINS, profileX, profileZ);

    std::vector<double> levels = detectStepLevels(profileZ, 5.0);

    std::cout << "  检测到 " << levels.size() << " 个台阶面高度:";
    for (double l : levels) std::cout << " " << l;
    std::cout << "\n";

    // 台阶高度 = 各台阶面高度 - 基准面高度（最低面）
    std::vector<double> stepHeights;
    if (levels.size() >= 2) {
        double baseZ = levels[0];
        for (size_t i = 1; i < levels.size(); ++i) {
            stepHeights.push_back(levels[i] - baseZ);
        }
    }

    bool step1OK = false, step2OK = false;
    if (stepHeights.size() >= 1) {
        double err1 = std::abs(stepHeights[0] - STEP1_GT_HEIGHT);
        step1OK = (err1 < TH_STEP_HEIGHT);
        if (!step1OK) allPassed = false;

        std::cout << std::setprecision(3);
        std::cout << "  台阶1高度: 测量 " << stepHeights[0]
                  << " mm, 真实 " << STEP1_GT_HEIGHT
                  << " mm, 误差 " << err1 << " mm ("
                  << std::setprecision(1) << (err1 / STEP1_GT_HEIGHT * 100.0)
                  << "%)  " << passFail(step1OK) << "\n";
    } else {
        std::cout << "  台阶1高度: [未能识别]  FAIL\n";
        allPassed = false;
    }

    if (stepHeights.size() >= 2) {
        double err2 = std::abs(stepHeights[1] - STEP2_GT_HEIGHT);
        step2OK = (err2 < TH_STEP_HEIGHT);
        if (!step2OK) allPassed = false;

        std::cout << std::setprecision(3);
        std::cout << "  台阶2高度: 测量 " << stepHeights[1]
                  << " mm, 真实 " << STEP2_GT_HEIGHT
                  << " mm, 误差 " << err2 << " mm ("
                  << std::setprecision(1) << (err2 / STEP2_GT_HEIGHT * 100.0)
                  << "%)  " << passFail(step2OK) << "\n";
    } else {
        std::cout << "  台阶2高度: [未能识别]  FAIL\n";
        allPassed = false;
    }

    // --------------------------------------------------
    // 6. 体积测量
    // --------------------------------------------------
    std::cout << "\n[体积测量] 计算物体体积...\n";

    // 真实体积计算：
    // 3 个等宽区域（基础面 0mm + 台阶1 10mm + 台阶2 20mm）
    double segWidth = BLOCK_WIDTH / 3.0;
    double segDepth = BLOCK_DEPTH;
    double gtVol = segWidth * segDepth * 0.0
                 + segWidth * segDepth * STEP1_GT_HEIGHT
                 + segWidth * segDepth * STEP2_GT_HEIGHT;

    // 使用 zMin 作为参考平面
    double zMin = std::numeric_limits<double>::max();
    for (const auto& p : reconCloud) {
        if (p.z() < zMin) zMin = p.z();
    }
    Eigen::Vector4d refPlane(0.0, 0.0, 1.0, -zMin);

    double measVol = lsc::PointCloudMeasurer::computeVolume(reconCloud, refPlane, 0.5);

    double volErrPct = (gtVol > 0.0) ? std::abs(measVol - gtVol) / gtVol * 100.0 : 0.0;
    bool volOK = (volErrPct < TH_VOLUME_PCT);
    if (!volOK) allPassed = false;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  测量体积: " << measVol << " mm^3\n";
    std::cout << "  真实体积: " << gtVol << " mm^3\n";
    std::cout << "  误差: " << std::setprecision(1) << volErrPct << "%  "
              << passFail(volOK) << "\n\n";

    // --------------------------------------------------
    // 综合判定
    // --------------------------------------------------
    std::cout << "========================================\n";
    std::cout << "  综合判定: " << passFail(allPassed) << "\n";
    std::cout << "========================================\n";

    return allPassed ? 0 : 1;
}
