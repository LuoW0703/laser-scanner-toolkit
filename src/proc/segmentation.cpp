#include "lsc/proc/segmentation.h"
#include "lsc/core/log.h"
#include <Eigen/SVD>
#include <random>
#include <cmath>
#include <algorithm>
#include <unordered_set>

namespace lsc {

// ============================================================================
// RANSAC 平面检测
//
// 标准 RANSAC 流程：
// 1. 随机选 3 点，计算平面参数 (n, D)
// 2. 统计内点（到平面距离 < distThresh）
// 3. 保留内点数最多的模型
// 4. 迭代次数自适应：N = log(1-confidence) / log(1 - w^3)，其中 w = 内点比例
// 5. 最后对全体内点做 SVD 精拟合
// ============================================================================

PointCloudSegmenter::PlaneModel PointCloudSegmenter::ransacPlane(
    const PointCloud& cloud,
    double distThresh,
    int maxIter,
    double confidence) {
    PlaneModel bestModel;
    bestModel.params = Eigen::Vector4d(0, 0, 1, 0);
    bestModel.rmsError = std::numeric_limits<double>::max();

    if (distThresh <= 0.0 || maxIter <= 0 ||
        confidence <= 0.0 || confidence >= 1.0) {
        LSC_ERROR("Segmentation")
            << "ransacPlane: invalid parameters, distThresh=" << distThresh
            << ", maxIter=" << maxIter
            << ", confidence=" << confidence;
        return bestModel;
    }
    if (cloud.size() < 3) {
        LSC_ERROR("Segmentation") << "ransacPlane: insufficient points: " << cloud.size();
        return bestModel;
    }

    const size_t N = cloud.size();

    // 随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dist(0, N - 1);

    size_t bestNumInliers = 0;
    int actualIter = 0;
    int adaptiveMaxIter = maxIter;

    for (int iter = 0; iter < adaptiveMaxIter; ++iter) {
        actualIter = iter + 1;

        // 随机选 3 个不同点
        size_t i0, i1, i2;
        std::unordered_set<size_t> used;
        i0 = dist(gen); used.insert(i0);
        do { i1 = dist(gen); } while (used.count(i1)); used.insert(i1);
        do { i2 = dist(gen); } while (used.count(i2));

        const Point3d& p0 = cloud[i0];
        const Point3d& p1 = cloud[i1];
        const Point3d& p2 = cloud[i2];

        // 计算法向量: n = (p1-p0) × (p2-p1)
        Eigen::Vector3d v1 = p1 - p0;
        Eigen::Vector3d v2 = p2 - p1;
        Eigen::Vector3d n = v1.cross(v2);

        double nLen = n.norm();
        if (nLen < 1e-12) {
            continue; // 三点共线，跳过
        }
        n /= nLen;

        // D = -n·p0
        double D = -n.dot(p0);

        // 法向量已经归一化，因此 |n·p+D| 就是点到候选平面的毫米距离。
        // RANSAC 的目标不是最小残差，而是先最大化阈值内的一致点数量。
        std::vector<size_t> currentInliers;
        for (size_t i = 0; i < N; ++i) {
            double pointDist = std::abs(n.dot(cloud[i]) + D);
            if (pointDist < distThresh) {
                currentInliers.push_back(i);
            }
        }

        // 更新最佳模型
        if (currentInliers.size() > bestNumInliers) {
            bestNumInliers = currentInliers.size();
            bestModel.params = Eigen::Vector4d(n.x(), n.y(), n.z(), D);
            bestModel.inlierIndices = currentInliers;

            // 若单次抽到全内点三元组的概率为 w^3，连续 N 次都失败的
            // 概率是 (1-w^3)^N。令其小于 1-confidence，可解得下式，
            // 内点率越高，所需迭代次数越少。
            double w = static_cast<double>(bestNumInliers) / N;
            if (w > 0.999) w = 0.999; // 防止 log(0) 情况
            if (w < 0.001) w = 0.001;
            double logProb = std::log(1.0 - confidence);
            double logDenom = std::log(1.0 - std::pow(w, 3.0));
            if (logDenom < 0.0) { // 防止除零
                int newMaxIter = static_cast<int>(std::ceil(logProb / logDenom));
                adaptiveMaxIter = std::min(newMaxIter, maxIter);
            }
        }
    }

    // 最终对全体内点做 SVD 精拟合
    if (bestModel.inlierIndices.size() >= 3) {
        // 提取内点
        PointCloud inlierPoints;
        inlierPoints.reserve(bestModel.inlierIndices.size());
        for (size_t idx : bestModel.inlierIndices) {
            inlierPoints.push_back(cloud[idx]);
        }

        // 计算质心
        Point3d centroid(0.0, 0.0, 0.0);
        for (const auto& p : inlierPoints) {
            centroid += p;
        }
        centroid /= static_cast<double>(inlierPoints.size());

        // 中心化
        Eigen::MatrixXd A(inlierPoints.size(), 3);
        for (size_t i = 0; i < inlierPoints.size(); ++i) {
            Point3d pc = inlierPoints[i] - centroid;
            A(i, 0) = pc.x();
            A(i, 1) = pc.y();
            A(i, 2) = pc.z();
        }

        // 随机三点模型只用于发现一致集。最终精度来自全部内点的
        // 最小二乘平面：中心化矩阵最小奇异值对应的右奇异向量，
        // 就是使点到平面平方距离之和最小的法向量。
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(
            A, Eigen::ComputeThinU | Eigen::ComputeThinV);

        Eigen::Vector3d normal = svd.matrixV().col(2);
        double D_refined = -normal.dot(centroid);

        // 确保法向量方向与之前一致
        Eigen::Vector3d oldNormal(bestModel.params.x(),
                                   bestModel.params.y(),
                                   bestModel.params.z());
        if (normal.dot(oldNormal) < 0.0) {
            normal = -normal;
            D_refined = -D_refined;
        }

        bestModel.params = Eigen::Vector4d(normal.x(), normal.y(), normal.z(), D_refined);

        // 计算 RMS 误差
        double rms = 0.0;
        for (const auto& p : inlierPoints) {
            double d = normal.dot(p) + D_refined;
            rms += d * d;
        }
        bestModel.rmsError = std::sqrt(rms / inlierPoints.size());
    }

    LSC_INFO("Segmentation") << "ransacPlane: iterations=" << actualIter
                             << ", inliers=" << bestModel.inlierIndices.size()
                             << "/" << N << ", rms=" << bestModel.rmsError << " mm";

    return bestModel;
}

// ============================================================================
// 根据平面模型分离内点和外点
// ============================================================================

void PointCloudSegmenter::separateInliers(const PointCloud& cloud,
                                           const PlaneModel& plane,
                                           PointCloud& outInliers,
                                           PointCloud& outOutliers) {
    outInliers.clear();
    outOutliers.clear();

    if (cloud.empty()) return;

    Eigen::Vector3d n(plane.params.x(), plane.params.y(), plane.params.z());
    double D = plane.params.w();

    // 归一化法向量
    double nLen = n.norm();
    if (nLen > 1e-12) {
        n /= nLen;
        D /= nLen;
    }

    // 精拟合后以 3*RMS 重新分类，等价于对近似高斯残差使用宽松的
    // 三西格玛范围。无有效 RMS 时回退到 0.1 mm，避免无限阈值。
    double thresh = plane.rmsError * 3.0;
    if (thresh < 1e-6) thresh = 0.1;      // 默认 0.1 mm

    for (size_t i = 0; i < cloud.size(); ++i) {
        double dist = std::abs(n.dot(cloud[i]) + D);
        if (dist < thresh) {
            outInliers.push_back(cloud[i]);
        } else {
            outOutliers.push_back(cloud[i]);
        }
    }
}

} // namespace lsc
