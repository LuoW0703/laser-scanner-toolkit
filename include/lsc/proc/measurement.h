#pragma once
#include "lsc/core/types.h"

namespace lsc {

/**
 * 点云测量工具
 *
 * 提供从点云计算几何属性（包围盒、体积等）的功能。
 */
class PointCloudMeasurer {
public:
    /** 包围盒结构体 */
    struct BoundingBox {
        Point3d minPoint = Point3d::Zero();
        Point3d maxPoint = Point3d::Zero();
        Point3d center = Point3d::Zero();
        Eigen::Vector3d dimensions = Eigen::Vector3d::Zero();
        // 列向量分别为局部 X/Y/Z 轴在世界坐标系中的单位方向。
        Eigen::Matrix3d axes = Eigen::Matrix3d::Identity();
    };

    /**
     * 计算轴对齐包围盒（AABB）
     *
     * 在点云原始坐标系下，分别取 X/Y/Z 方向的最小最大值。
     */
    static BoundingBox computeAABB(const PointCloud& cloud);

    /**
     * 计算有向包围盒（OBB）
     *
     * 通过 PCA 主成分分析找到点云主方向。axes 的列向量为 OBB
     * 局部坐标轴，dimensions 为对应轴向尺寸；minPoint/maxPoint 为
     * OBB 八个角点在世界坐标中的分量最小值和最大值。
     */
    static BoundingBox computeOBB(const PointCloud& cloud);

    /**
     * 投影栅格法体积测量
     *
     * 将点云投影到参考平面上，通过栅格划分计算每个栅格柱体的体积之和。
     * 适用于表面扫描点云的体积估算。
     *
     * @param cloud     输入点云
     * @param refPlane  参考平面参数 (A,B,C,D)，高度方向为平面法线方向
     * @param gridRes   栅格分辨率 (mm)
     * @return          总体积 (mm^3)
     */
    static double computeVolume(const PointCloud& cloud,
                                 const Eigen::Vector4d& refPlane,
                                 double gridRes = 0.1);
};

} // namespace lsc
