#pragma once
#include "lsc/core/types.h"

namespace lsc {

/**
 * 点云滤波器
 *
 * 提供常用的点云后处理滤波操作，所有方法均为静态方法。
 */
class PointCloudFilter {
public:
    /**
     * 统计离群点滤波（Statistical Outlier Removal）
     *
     * 对每个点计算其到 k 近邻的平均距离，假设距离分布满足高斯分布，
     * 剔除平均距离超过 mu + stdThresh * sigma 的点。
     *
     * @param cloud      输入点云
     * @param kNeighbors 近邻数 K（默认 50）
     * @param stdThresh  标准差倍数阈值（默认 1.0）
     * @return           滤波后的点云
     */
    static PointCloud statisticalFilter(const PointCloud& cloud,
                                         int kNeighbors = 50,
                                         double stdThresh = 1.0);

    /**
     * 体素下采样（Voxel Grid Downsampling）
     *
     * 将空间按 voxelSize 划分为立方体格网，每个格网内用重心替代所有点。
     * 在保持点云几何特征的同时减少点数。
     *
     * @param cloud     输入点云
     * @param voxelSize 体素边长 (mm)
     * @return          下采样后的点云
     */
    static PointCloud voxelDownsample(const PointCloud& cloud,
                                       double voxelSize);
};

} // namespace lsc
