#pragma once
#include "lsc/core/types.h"

namespace lsc {

/**
 * 像素坐标 --> 归一化射线方向（相机坐标系）
 *
 * dir = K^{-1} * [u, v, 1]^T
 * 返回的方向向量未归一化，|dir| >= 1（取决于像素位置和焦距）
 *
 * @param pixel  图像像素坐标 (col, row)
 * @param K      相机内参
 * @return       相机坐标系下的射线方向向量
 */
Eigen::Vector3d pixelToRay(const Point2d& pixel, const CameraIntrinsics& K);

/**
 * 射线-平面求交
 *
 * 射线: r(t) = t * dir, t > 0
 * 平面: n·P + D = 0
 * 解得 t = -D / (n·dir)
 * 3D 点 = t * dir
 *
 * @param pixel      图像像素坐标
 * @param K          相机内参
 * @param lightPlane 光平面参数
 * @param outPoint3D 输出 3D 点（相机坐标系）
 * @return           true=有效交点, false=平行或后方交点
 */
bool intersectRayPlane(
    const Point2d& pixel,
    const CameraIntrinsics& K,
    const Plane& lightPlane,
    Point3d& outPoint3D
);

/**
 * 批量射线-平面求交
 *
 * 等价于逐个调用 intersectRayPlane，但跳过无效交点。
 *
 * @param pixels     图像像素坐标集合
 * @param K          相机内参
 * @param lightPlane 光平面参数
 * @return           所有有效 3D 点（不含无效交点）
 */
std::vector<Point3d> intersectRayPlaneBatch(
    const std::vector<Point2d>& pixels,
    const CameraIntrinsics& K,
    const Plane& lightPlane
);

} // namespace lsc
