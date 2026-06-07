#pragma once
#include "lsc/core/types.h"

namespace lsc {

/**
 * 点云分割器
 *
 * 提供基于 RANSAC 的平面分割功能，用于从场景中提取参考平面或物体表面。
 */
class PointCloudSegmenter {
public:
    /** 平面模型结构体 */
    struct PlaneModel {
        Eigen::Vector4d params;        // 平面参数 (A, B, C, D)
        std::vector<size_t> inlierIndices; // 内点索引
        double rmsError = 0.0;         // 内点拟合均方根误差 (mm)
    };

    /**
     * RANSAC 平面检测
     *
     * 从点云中迭代随机采样三点拟合平面，返回内点数最多的平面模型。
     *
     * @param cloud       输入点云
     * @param distThresh  点到平面的距离阈值 (mm)，小于此值的点视为内点（默认 0.1）
     * @param maxIter     最大迭代次数（默认 1000）
     * @param confidence  置信度，用于自适应决定迭代次数（默认 0.99）
     * @return            检测到的平面模型
     */
    static PlaneModel ransacPlane(const PointCloud& cloud,
                                   double distThresh = 0.1,
                                   int maxIter = 1000,
                                   double confidence = 0.99);

    /**
     * 根据平面模型分离内点和外点
     *
     * @param cloud       输入点云
     * @param plane       RANSAC 检测到的平面模型
     * @param outInliers  输出内点（平面上的点）
     * @param outOutliers 输出外点（非平面的点）
     */
    static void separateInliers(const PointCloud& cloud, const PlaneModel& plane,
                                 PointCloud& outInliers, PointCloud& outOutliers);
};

} // namespace lsc
