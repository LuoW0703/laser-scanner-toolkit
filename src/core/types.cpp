#include "lsc/core/types.h"
#include <opencv2/calib3d.hpp>
#include <cmath>

namespace lsc {

// =====================================================================
// CameraIntrinsics 实现
// =====================================================================

cv::Mat CameraIntrinsics::matrix33() const {
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        fx,  0.0, cx,
        0.0, fy,  cy,
        0.0, 0.0, 1.0);
    return K;
}

cv::Mat CameraIntrinsics::distCoeffs() const {
    cv::Mat dist = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
    return dist;
}

CameraIntrinsics CameraIntrinsics::fromOpenCV(
    const cv::Mat& K, const cv::Mat& dist, int w, int h) {
    CameraIntrinsics c;
    c.fx       = K.at<double>(0, 0);
    c.fy       = K.at<double>(1, 1);
    c.cx       = K.at<double>(0, 2);
    c.cy       = K.at<double>(1, 2);
    c.k1       = dist.at<double>(0, 0);
    c.k2       = dist.at<double>(0, 1);
    c.p1       = dist.at<double>(0, 2);
    c.p2       = dist.at<double>(0, 3);
    c.k3       = dist.at<double>(0, 4);
    c.imgWidth = w;
    c.imgHeight = h;
    return c;
}

// =====================================================================
// Plane 实现
// =====================================================================

void Plane::normalize() {
    double len = std::sqrt(A * A + B * B + C * C);
    if (len > 1e-12) {
        A /= len;
        B /= len;
        C /= len;
        D /= len;
    }
}

double Plane::distanceToPoint(const Point3d& p) const {
    // 点到平面的有符号距离: (Ax + By + Cz + D) / sqrt(A^2 + B^2 + C^2)
    double len = std::sqrt(A * A + B * B + C * C);
    if (len < 1e-12) return 0.0;
    return std::abs(A * p.x() + B * p.y() + C * p.z() + D) / len;
}

} // namespace lsc
