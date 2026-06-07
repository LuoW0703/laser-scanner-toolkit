#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include <cstdint>

namespace lsc {

// ===== 基础点类型（使用 Eigen，方便数学运算）=====
using Point2d = Eigen::Vector2d;
using Point3d = Eigen::Vector3d;
using PointCloud = std::vector<Point3d>;

// ===== 相机内参 =====
struct CameraIntrinsics {
    double fx = 0.0, fy = 0.0;   // 焦距（像素）
    double cx = 0.0, cy = 0.0;   // 主点（像素）
    double k1 = 0.0, k2 = 0.0, k3 = 0.0; // 径向畸变
    double p1 = 0.0, p2 = 0.0;   // 切向畸变
    int imgWidth = 0, imgHeight = 0;

    // 转换为 OpenCV 矩阵
    cv::Mat matrix33() const;
    cv::Mat distCoeffs() const;

    // 从 OpenCV 矩阵构造
    static CameraIntrinsics fromOpenCV(const cv::Mat& K, const cv::Mat& dist,
                                        int w, int h);
};

// ===== 光平面：Ax + By + Cz + D = 0 =====
struct Plane {
    double A = 0.0, B = 0.0, C = 0.0, D = 0.0;

    Eigen::Vector3d normal() const { return {A, B, C}; }
    void normalize();
    double distanceToPoint(const Point3d& p) const;
};

// ===== 单次扫描线 =====
struct ScanLine {
    std::vector<Point2d> laserPoints; // 激光线亚像素坐标（图像坐标系）
    double motionPosition = 0.0;      // 该扫描线对应的移动轴位置 (mm)
    // 位置 0 mm 同样是有效编码器读数，不能再用数值是否为零判断
    // “是否提供位置”。旧调用方未设置此标志时才回退到固定步长。
    bool hasMotionPosition = false;
};

// ===== 标定结果集 =====
struct CalibrationResult {
    CameraIntrinsics camera;
    Plane            lightPlane;
    Eigen::Vector3d  motionAxis;
    double           rmsCamera = 0.0; // 相机标定重投影误差 (px)
    double           rmsPlane  = 0.0; // 光平面拟合残差 (mm)
    double           rmsAxis   = 0.0; // 移动轴拟合残差 (mm)
};

// ===== 简易 3D 网格（用于定义被测物体模型）=====
struct TriangleMesh {
    std::vector<Point3d> vertices;
    std::vector<std::array<int, 3>> faces;

    bool empty() const { return vertices.empty(); }
};

} // namespace lsc
