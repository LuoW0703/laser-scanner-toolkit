#include "lsc/proc/measurement.h"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <limits>
#include <map>
#include <tuple>
#include <vector>
#include <cmath>

namespace lsc {

// ============================================================================
// 计算轴对齐包围盒（AABB）
//
// 在点云原始坐标系下，分别对 X/Y/Z 取最小最大值。
// ============================================================================

PointCloudMeasurer::BoundingBox PointCloudMeasurer::computeAABB(const PointCloud& cloud) {
    BoundingBox box;
    if (cloud.empty()) {
        return box;
    }

    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double minZ = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    double maxZ = -std::numeric_limits<double>::max();

    for (const auto& p : cloud) {
        if (p.x() < minX) minX = p.x();
        if (p.y() < minY) minY = p.y();
        if (p.z() < minZ) minZ = p.z();
        if (p.x() > maxX) maxX = p.x();
        if (p.y() > maxY) maxY = p.y();
        if (p.z() > maxZ) maxZ = p.z();
    }

    box.minPoint = Point3d(minX, minY, minZ);
    box.maxPoint = Point3d(maxX, maxY, maxZ);
    box.center = Point3d((minX + maxX) / 2.0, (minY + maxY) / 2.0, (minZ + maxZ) / 2.0);
    box.dimensions = Eigen::Vector3d(maxX - minX, maxY - minY, maxZ - minZ);
    box.axes = Eigen::Matrix3d::Identity();

    return box;
}

// ============================================================================
// 计算有向包围盒（OBB）
//
// 通过 PCA 主成分分析找到点云的主方向。
// 流程：
// 1. 中心化点云
// 2. 计算协方差矩阵
// 3. 特征值分解，特征向量 = 主轴方向
// 4. 将点投影到主轴坐标系中取 min/max 得到包围盒尺寸
// ============================================================================

PointCloudMeasurer::BoundingBox PointCloudMeasurer::computeOBB(const PointCloud& cloud) {
    BoundingBox box;
    if (cloud.empty()) {
        return box;
    }

    const size_t N = cloud.size();

    // 1. 计算质心
    Point3d centroid(0.0, 0.0, 0.0);
    for (const auto& p : cloud) {
        centroid += p;
    }
    centroid /= static_cast<double>(N);

    // 2. 中心化并构造协方差矩阵。协方差描述点云沿各方向的离散程度；
    // 最大特征值方向是点云最长主轴，最小特征值方向通常接近表面法向。
    // 协方差矩阵 C = (1/N) * Σ (p_i - centroid)(p_i - centroid)^T
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : cloud) {
        Eigen::Vector3d pc = p - centroid;
        cov += pc * pc.transpose();
    }
    cov /= static_cast<double>(N);

    // 3. 特征值分解
    // SelfAdjointEigenSolver 返回按特征值升序排列的特征向量
    // 特征向量 = 主轴方向
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(cov);

    if (eigenSolver.info() != Eigen::Success) {
        // 特征值分解失败，回退到 AABB
        return computeAABB(cloud);
    }

    // 特征向量矩阵（列向量为各主轴方向），从最大特征值到最小排列
    // eigenSolver 返回升序，需要反转
    Eigen::Matrix3d axes;
    axes.col(0) = eigenSolver.eigenvectors().col(2); // 最大特征值 → X 轴
    axes.col(1) = eigenSolver.eigenvectors().col(1); // 中等特征值 → Y 轴
    axes.col(2) = eigenSolver.eigenvectors().col(0); // 最小特征值 → Z 轴

    // 4. 将点投影到主轴坐标系中，取边界
    double minProj[3] = { std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max() };
    double maxProj[3] = { -std::numeric_limits<double>::max(),
                          -std::numeric_limits<double>::max(),
                          -std::numeric_limits<double>::max() };

    for (const auto& p : cloud) {
        Eigen::Vector3d pc = p - centroid;
        for (int d = 0; d < 3; ++d) {
            double proj = pc.dot(axes.col(d));
            if (proj < minProj[d]) minProj[d] = proj;
            if (proj > maxProj[d]) maxProj[d] = proj;
        }
    }

    // 包围盒尺寸
    box.dimensions = Eigen::Vector3d(
        maxProj[0] - minProj[0],
        maxProj[1] - minProj[1],
        maxProj[2] - minProj[2]);

    // 包围盒中心（在主轴坐标系中）
    Eigen::Vector3d centerProj(
        (minProj[0] + maxProj[0]) / 2.0,
        (minProj[1] + maxProj[1]) / 2.0,
        (minProj[2] + maxProj[2]) / 2.0);

    box.center = centroid + axes * centerProj;

    box.axes = axes;

    // 计算八个 OBB 角点在世界坐标中的分量边界。
    box.minPoint = Point3d::Constant(std::numeric_limits<double>::max());
    box.maxPoint = Point3d::Constant(-std::numeric_limits<double>::max());
    const Eigen::Vector3d halfSize = box.dimensions / 2.0;
    for (int sx : {-1, 1}) {
        for (int sy : {-1, 1}) {
            for (int sz : {-1, 1}) {
                const Eigen::Vector3d localCorner(
                    sx * halfSize.x(), sy * halfSize.y(), sz * halfSize.z());
                const Point3d worldCorner = box.center + axes * localCorner;
                box.minPoint = box.minPoint.cwiseMin(worldCorner);
                box.maxPoint = box.maxPoint.cwiseMax(worldCorner);
            }
        }
    }

    return box;
}

// ============================================================================
// 投影栅格法体积测量
//
// 将点云投影到参考平面上，通过 2D 栅格划分计算每个栅格柱体的体积之和。
// 适用于表面扫描点云的体积估算。
// ============================================================================

double PointCloudMeasurer::computeVolume(const PointCloud& cloud,
                                          const Eigen::Vector4d& refPlane,
                                          double gridRes) {
    if (cloud.empty() || gridRes <= 0.0) {
        return 0.0;
    }

    // 提取平面参数
    Eigen::Vector3d n(refPlane.x(), refPlane.y(), refPlane.z());
    double D = refPlane.w();

    // 归一化法向量
    double nLen = n.norm();
    if (nLen < 1e-12) {
        return 0.0;
    }
    n /= nLen;
    D /= nLen;

    // 构建投影平面上的局部 2D 坐标系
    // 选择第一个基向量 e1：与 n 正交的任意方向
    Eigen::Vector3d e1;
    if (std::abs(n.x()) < 0.9) {
        e1 = n.cross(Eigen::Vector3d(1, 0, 0));
    } else {
        e1 = n.cross(Eigen::Vector3d(0, 1, 0));
    }
    e1.normalize();
    Eigen::Vector3d e2 = n.cross(e1).normalized();

    // 将所有点投影到参考平面的局部 2D 栅格。e1/e2 只负责定位柱体，
    // 高度 h=n·p+D 则沿真实平面法向计算，因此参考平面可以任意倾斜。
    // 投影坐标: (p·e1, p·e2)
    // 高度: 点到平面的有符号距离: h = n·p + D

    // 栅格哈希: (gridX, gridY) → (高度的列表)
    std::map<std::tuple<int, int>, std::vector<double>> gridHeights;

    for (const auto& p : cloud) {
        double u = p.dot(e1);
        double v = p.dot(e2);
        double h = n.dot(p) + D; // 有符号距离（正值 = 在法向量方向上方）

        int gx = static_cast<int>(std::floor(u / gridRes));
        int gy = static_cast<int>(std::floor(v / gridRes));

        gridHeights[{gx, gy}].push_back(h);
    }

    // 每个栅格代表底面积 gridRes^2 的小柱体。使用中位高度而非最大值
    // 可抑制飞点和边缘毛刺；仅累加参考平面法向一侧的正高度。
    double gridArea = gridRes * gridRes;
    double volume = 0.0;

    for (auto& kv : gridHeights) {
        auto& heights = kv.second;
        if (heights.empty()) continue;

        // 取中位数高度（稳健估计）
        std::sort(heights.begin(), heights.end());
        double medianH = heights[heights.size() / 2];

        // 只计算正高度（参考平面以上的体积）
        if (medianH > 0.0) {
            volume += gridArea * medianH;
        }
    }

    return volume;
}

} // namespace lsc
