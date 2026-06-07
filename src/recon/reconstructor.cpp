#include "lsc/recon/reconstructor.h"
#include "lsc/core/ray_plane.h"
#include <opencv2/imgcodecs.hpp>

namespace lsc {

Reconstructor::Reconstructor(const CameraIntrinsics& K, const Plane& lightPlane,
                             const Eigen::Vector3d& motionAxis)
    : m_K(K), m_lightPlane(lightPlane), m_motionAxis(motionAxis) {}

std::vector<Point3d> Reconstructor::reconstructSingleLine(
    const std::vector<Point2d>& laserPoints) const {
    // 单帧中每个亚像素中心对应一条相机射线。所有射线与已标定光平面
    // 相交后，得到当前扫描截面在相机坐标系中的 3D 轮廓。
    return intersectRayPlaneBatch(laserPoints, m_K, m_lightPlane);
}

PointCloud Reconstructor::reconstruct(
    const std::vector<ScanLine>& scanLines, double motionStep) const {
    PointCloud result;
    for (size_t i = 0; i < scanLines.size(); ++i) {
        auto line3D = intersectRayPlaneBatch(
            scanLines[i].laserPoints, m_K, m_lightPlane);

        // motionPosition 是采集时记录的绝对位置，可能为负数或恰好为
        // 0 mm。显式有效标志避免把合法零位置错误替换成 i*motionStep。
        const double offset = scanLines[i].hasMotionPosition
                                  ? scanLines[i].motionPosition
                                  : i * motionStep;

        // 将每条位于相机坐标系的截面沿标定移动轴放置到统一坐标系，
        // 再拼接成完整点云。m_motionAxis 必须是单位向量，否则 offset
        // 的毫米量纲会被缩放。
        for (auto& pt : line3D) {
            pt += offset * m_motionAxis;
        }
        result.insert(result.end(), line3D.begin(), line3D.end());
    }
    return result;
}

PointCloud Reconstructor::reconstructFromImages(
    const std::vector<std::string>& imagePaths,
    double motionStep,
    LaserMethod laserMethod,
    double laserThresh,
    double laserSigma) {
    PointCloud result;
    for (size_t i = 0; i < imagePaths.size(); ++i) {
        cv::Mat img = cv::imread(imagePaths[i], cv::IMREAD_GRAYSCALE);
        if (img.empty()) continue;
        // 此便捷接口把“图像读取、中心线提取、三角测量、运动补偿”
        // 串在一起；需要保存中间证据或使用编码器位置时，应使用
        // reconstruct(ScanLine) 的显式接口。
        auto pts2D = m_extractor.extract(
            img, laserMethod, laserThresh, laserSigma);
        auto pts3D = intersectRayPlaneBatch(pts2D, m_K, m_lightPlane);
        for (auto& pt : pts3D) {
            pt += i * motionStep * m_motionAxis;
        }
        result.insert(result.end(), pts3D.begin(), pts3D.end());
    }
    return result;
}

} // namespace lsc
