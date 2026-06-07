#pragma once
#include "lsc/core/types.h"
#include "lsc/core/laser_line.h"

namespace lsc {

/**
 * 3D 重建器 —— 将激光线图像坐标转换为 3D 点云
 *
 * 核心流程：
 * 1. 取像素坐标 --> 射线（pixelToRay）
 * 2. 射线与光平面求交 --> 3D 点（intersectRayPlane）
 * 3. 沿移动轴方向平移 --> 完整扫描点云
 */
class Reconstructor {
public:
    /**
     * 构造函数
     * @param K          已标定的相机内参
     * @param lightPlane 已标定的光平面
     * @param motionAxis 已标定的移动轴方向
     */
    Reconstructor(const CameraIntrinsics& K, const Plane& lightPlane,
                  const Eigen::Vector3d& motionAxis);

    /**
     * 单条激光线重建（无运动，激光线 3D 点在同一截面上）
     * @param laserPoints 单行激光线亚像素坐标
     * @return            3D 点集（相机坐标系）
     */
    std::vector<Point3d> reconstructSingleLine(
        const std::vector<Point2d>& laserPoints) const;

    /**
     * 多线拼接重建
     *
     * @param scanLines  多条扫描线（每条包含激光像素坐标和移动轴位置）
     * @param motionStep 移动轴步长 (mm/线)，仅在 scanLines[i].motionPosition 未设置时使用
     * @return           完整 3D 点云（相机坐标系）
     */
    PointCloud reconstruct(
        const std::vector<ScanLine>& scanLines, double motionStep) const;

    /**
     * 从图像文件直接重建
     *
     * 对每张图像提取激光线、求交、沿轴平移。
     *
     * @param imagePaths  扫描图像文件路径（按扫描顺序排列）
     * @param motionStep  相邻图像的移动轴步长 (mm)
     * @param laserMethod 激光线提取方法
     * @param laserThresh 激光线提取阈值
     * @param laserSigma  激光线提取 sigma
     * @return            完整 3D 点云（相机坐标系）
     */
    PointCloud reconstructFromImages(
        const std::vector<std::string>& imagePaths,
        double motionStep,
        LaserMethod laserMethod = LaserMethod::GRAY_CENTROID,
        double laserThresh = 50.0, double laserSigma = 1.5);

    /** 返回当前使用的相机内参（只读） */
    const CameraIntrinsics& getCamera() const { return m_K; }

private:
    CameraIntrinsics m_K;
    Plane            m_lightPlane;
    Eigen::Vector3d  m_motionAxis;
    LaserLineExtractor m_extractor;
};

} // namespace lsc
