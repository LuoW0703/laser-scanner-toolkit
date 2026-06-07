/**
 * @file    demo_processing.cpp
 * @brief   点云处理演示：加载 PLY 点云 → 滤波 → 下采样 → RANSAC 平面 → 测量 → 输出结果
 *
 * 如果未指定输入文件，则先用 SimulatedScanner 生成扫描数据并重建点云，
 * 然后对其进行完整的后处理流水线。
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <limits>

#include "lsc/core/types.h"
#include "lsc/core/io_utils.h"
#include "lsc/sim/simulated_scanner.h"
#include "lsc/recon/reconstructor.h"
#include "lsc/proc/filter.h"
#include "lsc/proc/segmentation.h"
#include "lsc/proc/measurement.h"

int main(int argc, char* argv[]) {
    std::cout << "========================================\n";
    std::cout << "  点云处理演示\n";
    std::cout << "========================================\n\n";

    lsc::PointCloud inputCloud;

    // --------------------------------------------------
    // 1. 获取点云数据
    // --------------------------------------------------
    if (argc >= 2) {
        // 从文件加载
        std::string filePath = argv[1];
        std::cout << "[加载] 从文件加载点云: " << filePath << "\n";

        bool loaded = false;
        if (filePath.find(".ply") != std::string::npos ||
            filePath.find(".PLY") != std::string::npos) {
            loaded = lsc::loadPLY(filePath, inputCloud);
        } else if (filePath.find(".xyz") != std::string::npos ||
                   filePath.find(".XYZ") != std::string::npos) {
            loaded = lsc::loadXYZ(filePath, inputCloud);
        }

        if (!loaded) {
            std::cerr << "[错误] 无法加载点云文件: " << filePath << "\n";
            return 1;
        }
        std::cout << "  加载点数: " << inputCloud.size() << "\n\n";

    } else {
        // 自动生成模拟点云
        std::cout << "[生成] 无输入文件，自动生成模拟扫描点云...\n";

        lsc::sim::SimulatedScanner::Config cfg;
        lsc::sim::SimulatedScanner scanner(cfg);

        const lsc::CameraIntrinsics& K_gt  = scanner.getIntrinsics();
        const lsc::Plane&            P_gt  = cfg.lightPlane;
        const Eigen::Vector3d&       Ax_gt = cfg.motionAxis;

        lsc::Reconstructor recon(K_gt, P_gt, Ax_gt);

        lsc::sim::StepBlock block;
        block.width       = 60.0;
        block.depth       = 40.0;
        block.stepHeights = {10.0, 20.0};

        auto [scanImages, gtCloud] = scanner.generateScan(block);

        std::cout << "  扫描图像: " << scanImages.size() << " 帧\n";

        // 提取激光线并重建
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

            if ((i + 1) % 40 == 0 || i + 1 == scanImages.size()) {
                std::cout << "  ... 提取中 " << (i + 1) << "/"
                          << scanImages.size() << "\n";
            }
        }

        inputCloud = recon.reconstruct(scanLines, cfg.defaultStep);
        std::cout << "  重建点数: " << inputCloud.size() << "\n\n";

        // 保存原始点云
        lsc::savePLY("output/scan_result.ply", inputCloud);
        std::cout << "  原始点云已保存到: output/scan_result.ply\n\n";
    }

    if (inputCloud.empty()) {
        std::cerr << "[错误] 点云为空！\n";
        return 1;
    }

    // --------------------------------------------------
    // 2. 统计离群点滤波
    // --------------------------------------------------
    std::cout << "[滤波] 统计离群点去除 (k=50, sigma=1.0)...\n";

    size_t beforeFilter = inputCloud.size();
    lsc::PointCloud filtered = lsc::PointCloudFilter::statisticalFilter(
        inputCloud, 50, 1.0);
    size_t afterFilter = filtered.size();

    double removedPct = (beforeFilter > 0)
        ? (1.0 - static_cast<double>(afterFilter) / beforeFilter) * 100.0
        : 0.0;

    std::cout << "  滤波前点数: " << beforeFilter << "\n";
    std::cout << "  滤波后点数: " << afterFilter << "\n";
    std::cout << "  移除离群点: " << (beforeFilter - afterFilter)
              << " (" << std::fixed << std::setprecision(1) << removedPct << "%)\n\n";

    // --------------------------------------------------
    // 3. 体素下采样
    // --------------------------------------------------
    std::cout << "[下采样] 体素栅格下采样 (voxel=0.2mm)...\n";

    size_t beforeDS = filtered.size();
    lsc::PointCloud downsampled = lsc::PointCloudFilter::voxelDownsample(
        filtered, 0.2);
    size_t afterDS = downsampled.size();

    std::cout << "  下采样前: " << beforeDS << " 点\n";
    std::cout << "  下采样后: " << afterDS << " 点\n";
    std::cout << "  压缩比:   " << std::fixed << std::setprecision(1)
              << (static_cast<double>(beforeDS) / afterDS) << ":1\n\n";

    // --------------------------------------------------
    // 4. RANSAC 平面分割
    // --------------------------------------------------
    std::cout << "[分割] RANSAC 平面检测 (阈值=0.5mm, 迭代=1000)...\n";

    auto planeModel = lsc::PointCloudSegmenter::ransacPlane(
        downsampled, 0.5, 1000, 0.99);

    std::cout << "  内点数: " << planeModel.inlierIndices.size()
              << " / " << downsampled.size() << "\n";
    std::cout << "  平面参数: (" << std::fixed << std::setprecision(4)
              << planeModel.params(0) << ", "
              << planeModel.params(1) << ", "
              << planeModel.params(2) << ", "
              << planeModel.params(3) << ")\n";
    std::cout << "  平面 RMS: " << planeModel.rmsError << " mm\n\n";

    // 分离内点和外点
    lsc::PointCloud inliers, outliers;
    lsc::PointCloudSegmenter::separateInliers(downsampled, planeModel,
                                               inliers, outliers);
    std::cout << "  平面内点: " << inliers.size()
              << ", 非平面点: " << outliers.size() << "\n\n";

    // --------------------------------------------------
    // 5. 测量
    // --------------------------------------------------
    std::cout << "[测量] 几何属性计算...\n";

    // 轴对齐包围盒
    auto aabb = lsc::PointCloudMeasurer::computeAABB(downsampled);
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  AABB:\n";
    std::cout << "    尺寸: " << aabb.dimensions(0) << " x "
              << aabb.dimensions(1) << " x " << aabb.dimensions(2) << " mm\n";
    std::cout << "    中心: (" << aabb.center.x() << ", "
              << aabb.center.y() << ", " << aabb.center.z() << ")\n";

    // 有向包围盒
    auto obb = lsc::PointCloudMeasurer::computeOBB(downsampled);
    std::cout << "  OBB:\n";
    std::cout << "    尺寸: " << obb.dimensions(0) << " x "
              << obb.dimensions(1) << " x " << obb.dimensions(2) << " mm\n";
    std::cout << "    中心: (" << obb.center.x() << ", "
              << obb.center.y() << ", " << obb.center.z() << ")\n";

    // 体积估算
    double volume = lsc::PointCloudMeasurer::computeVolume(
        downsampled, planeModel.params, 0.2);
    std::cout << "\n  体积 (栅格法, 参考平面): " << std::fixed
              << std::setprecision(2) << volume << " mm^3\n";

    // --------------------------------------------------
    // 6. 保存结果
    // --------------------------------------------------
    std::cout << "\n[保存] 写入处理结果...\n";

    lsc::savePLY("output/scan_result_filtered.ply", filtered);
    std::cout << "  滤波后点云: output/scan_result_filtered.ply\n";

    lsc::savePLY("output/scan_result_plane.ply", inliers);
    std::cout << "  平面内点:   output/scan_result_plane.ply\n";

    std::cout << "\n========================================\n";
    std::cout << "  点云处理演示完成\n";
    std::cout << "========================================\n";

    return 0;
}
