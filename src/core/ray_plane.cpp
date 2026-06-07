#include "lsc/core/ray_plane.h"
#include <cmath>

namespace lsc {

Eigen::Vector3d pixelToRay(const Point2d& pixel, const CameraIntrinsics& K) {
    // 针孔模型：
    //   u = fx * X/Z + cx, v = fy * Y/Z + cy
    // 令 Z=1，可得到从相机光心出发、穿过像素的未归一化射线方向。
    // 这里不归一化是有意的：后续参数 t 直接等于交点的相机 Z 深度。
    double u = (pixel.x() - K.cx) / K.fx;
    double v = (pixel.y() - K.cy) / K.fy;
    return {u, v, 1.0};
}

bool intersectRayPlane(
    const Point2d& pixel,
    const CameraIntrinsics& K,
    const Plane& lightPlane,
    Point3d& outPoint3D) {
    Eigen::Vector3d dir = pixelToRay(pixel, K);
    Eigen::Vector3d n = lightPlane.normal();

    // 射线 P(t)=t*dir 代入平面 n·P+D=0：
    //   t = -D / (n·dir)
    // 分母接近 0 时射线与平面平行；t<=0 时交点位于相机后方，
    // 两种情况都不能形成有效三角测量点。
    double denom = n.dot(dir);
    if (std::abs(denom) < 1e-12) {
        return false; // 射线与平面平行
    }
    double t = -lightPlane.D / denom;
    if (t <= 0.0) {
        return false; // 交点在相机后方
    }
    outPoint3D = t * dir;
    return true;
}

std::vector<Point3d> intersectRayPlaneBatch(
    const std::vector<Point2d>& pixels,
    const CameraIntrinsics& K,
    const Plane& lightPlane) {
    std::vector<Point3d> result;
    result.reserve(pixels.size());
    // 无效像素不会在结果中占位，因此调用方得到的是紧凑有效点集。
    // 如果业务需要保持像素与三维点的一一索引，应改用 optional/result
    // 形式，而不能依赖此批处理函数的下标。
    for (const auto& p : pixels) {
        Point3d pt;
        if (intersectRayPlane(p, K, lightPlane, pt)) {
            result.push_back(pt);
        }
    }
    return result;
}

} // namespace lsc
