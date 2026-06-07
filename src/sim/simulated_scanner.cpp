#include "lsc/sim/simulated_scanner.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lsc {
namespace sim {

// ============================================================
// 构造与初始化
// ============================================================

SimulatedScanner::SimulatedScanner()
    : SimulatedScanner(Config{})
{
}

SimulatedScanner::SimulatedScanner(const Config& cfg)
    : cfg_(cfg)
    , rng_(std::mt19937::default_seed)
{
    initIntrinsics();
}

void SimulatedScanner::initIntrinsics() {
    intrinsics_.imgWidth  = cfg_.imgWidth;
    intrinsics_.imgHeight = cfg_.imgHeight;

    // fx = fy = 焦距 / 像元尺寸
    intrinsics_.fx = cfg_.focalLength / cfg_.pixelSize;
    intrinsics_.fy = intrinsics_.fx;

    // 主点位于图像中心
    intrinsics_.cx = cfg_.imgWidth  / 2.0;
    intrinsics_.cy = cfg_.imgHeight / 2.0;

    // 径向畸变（仅使用 k1, k2，其余置零）
    intrinsics_.k1 = cfg_.k1;
    intrinsics_.k2 = cfg_.k2;
    intrinsics_.k3 = 0.0;
    intrinsics_.p1 = 0.0;
    intrinsics_.p2 = 0.0;
}

// ============================================================
// 投影函数
// ============================================================

Point2d SimulatedScanner::project(const Point3d& p) const {
    if (p.z() <= 0.0) {
        // 点在相机后方，返回无效坐标
        return Point2d(-1.0, -1.0);
    }

    // 归一化坐标
    double xn = p.x() / p.z();
    double yn = p.y() / p.z();

    // 径向畸变（k1, k2，忽略 k3 和高阶项）
    double r2 = xn * xn + yn * yn;
    double r4 = r2 * r2;
    double radial = 1.0 + intrinsics_.k1 * r2 + intrinsics_.k2 * r4;

    // 切向畸变（p1=p2=0，这里保留完整公式以便扩展）
    double p1 = intrinsics_.p1;
    double p2 = intrinsics_.p2;
    double xd = xn * radial + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn);
    double yd = yn * radial + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn;

    // 像素坐标
    double u = intrinsics_.fx * xd + intrinsics_.cx;
    double v = intrinsics_.fy * yd + intrinsics_.cy;

    return Point2d(u, v);
}

std::vector<Point2d> SimulatedScanner::projectBatch(const std::vector<Point3d>& pts) const {
    std::vector<Point2d> result;
    result.reserve(pts.size());
    for (const auto& p : pts) {
        result.push_back(project(p));
    }
    return result;
}

// ============================================================
// 棋盘格
// ============================================================

std::vector<Point3d> SimulatedScanner::getChessboardCorners3d() const {
    std::vector<Point3d> corners;
    int cols = cfg_.chessboardCols;   // 内角点列数 9
    int rows = cfg_.chessboardRows;   // 内角点行数 6
    double s  = cfg_.squareSize;      // 方格尺寸 15mm
    double bw = (cols + 1) * s;       // 整板宽度 150mm
    double bh = (rows + 1) * s;       // 整板高度 105mm

    for (int j = 0; j < rows; ++j) {
        for (int i = 0; i < cols; ++i) {
            // 内角点位于整板的 (s, s) 到 (cols*s, rows*s)
            // 将棋盘格中心置于原点
            double x = (i + 1) * s - bw / 2.0;
            double y = (j + 1) * s - bh / 2.0;
            corners.emplace_back(x, y, 0.0);
        }
    }
    return corners;
}

cv::Mat SimulatedScanner::renderChessboard(const Eigen::Isometry3d& pose) {
    const int nSquaresX = cfg_.chessboardCols + 1; // 方格列数（如 10）
    const int nSquaresY = cfg_.chessboardRows + 1; // 方格行数（如 7）
    const double s = cfg_.squareSize;
    const double bw = nSquaresX * s; // 整板总宽度（如 150 mm）
    const double bh = nSquaresY * s; // 整板总高度（如 105 mm）
    const double halfBw = bw / 2.0;
    const double halfBh = bh / 2.0;
    const double border = s * 0.5;
    const double renderHalfBw = halfBw + border;
    const double renderHalfBh = halfBh + border;

    // 背景：环境光灰度
    cv::Mat image(cfg_.imgHeight, cfg_.imgWidth, CV_8UC1,
                  cv::Scalar(static_cast<int>(cfg_.ambientLight)));

    // 提取姿态的旋转矩阵与平移向量
    Eigen::Matrix3d R = pose.rotation();
    Eigen::Vector3d  t = pose.translation();

    // 棋盘格平面法向量（棋盘格坐标系 Z 轴在相机坐标系中的方向）
    Eigen::Vector3d N = R.col(2);

    // 平面方程参数 N·P + D = 0，其中 D = -N·t
    double D = -N.dot(t);

    // 相机内参预取
    const double fx = intrinsics_.fx;
    const double fy = intrinsics_.fy;
    const double cx = intrinsics_.cx;
    const double cy = intrinsics_.cy;

    // === 计算棋盘格在图像上的包围盒，以限定逐像素遍历范围 ===
    // 使用所有 (nSquaresX+1)×(nSquaresY+1) 个网格角点的投影来求包围盒，
    // 确保畸变引起的桶形/枕形变化也被包含在内。
    double umin = static_cast<double>(cfg_.imgWidth);
    double umax = 0.0;
    double vmin = static_cast<double>(cfg_.imgHeight);
    double vmax = 0.0;
    bool anyValid = false;

    for (int r = 0; r <= nSquaresY; ++r) {
        for (int c = 0; c <= nSquaresX; ++c) {
            // 网格角点在棋盘格局部坐标系中的坐标
            Eigen::Vector3d cornerLocal(c * s - halfBw, r * s - halfBh, 0.0);
            Eigen::Vector3d cornerCam = pose * cornerLocal;
            if (cornerCam.z() <= 0.0) continue; // 在相机后方，跳过

            Point2d cornerImg = project(cornerCam); // 使用含畸变的投影
            if (cornerImg.x() >= 0.0 && cornerImg.y() >= 0.0) {
                umin = std::min(umin, cornerImg.x());
                umax = std::max(umax, cornerImg.x());
                vmin = std::min(vmin, cornerImg.y());
                vmax = std::max(vmax, cornerImg.y());
                anyValid = true;
            }
        }
    }

    // Include the white backing border in the rasterization bounds.
    for (double x : {-renderHalfBw, renderHalfBw}) {
        for (double y : {-renderHalfBh, renderHalfBh}) {
            const Eigen::Vector3d cornerCam = pose * Eigen::Vector3d(x, y, 0.0);
            if (cornerCam.z() <= 0.0) {
                continue;
            }
            const Point2d cornerImg = project(cornerCam);
            umin = std::min(umin, cornerImg.x());
            umax = std::max(umax, cornerImg.x());
            vmin = std::min(vmin, cornerImg.y());
            vmax = std::max(vmax, cornerImg.y());
            anyValid = true;
        }
    }

    if (!anyValid) {
        // 棋盘格完全不可见，直接返回背景图
        return image;
    }

    // 扩展几个像素避免边界遗漏，然后裁切到图像范围内
    const double margin = 3.0;
    int u0 = std::max(0, static_cast<int>(std::floor(umin - margin)));
    int u1 = std::min(cfg_.imgWidth - 1, static_cast<int>(std::ceil(umax + margin)));
    int v0 = std::max(0, static_cast<int>(std::floor(vmin - margin)));
    int v1 = std::min(cfg_.imgHeight - 1, static_cast<int>(std::ceil(vmax + margin)));

    // === 逐像素光线求交（忽略畸变以简化反投影） ===
    for (int v = v0; v <= v1; ++v) {
        // 指向当前行的数据指针，避免重复调用 at()
        uchar* rowPtr = image.ptr<uchar>(v);

        for (int u = u0; u <= u1; ++u) {
            // 1. 反投影到归一化相机坐标
            double xn = (static_cast<double>(u) - cx) / fx;
            double yn = (static_cast<double>(v) - cy) / fy;

            // 2. 光线方向（相机坐标系）
            Eigen::Vector3d rayDir(xn, yn, 1.0);

            // 3. 求光线与棋盘格平面的交点
            double denom = N.dot(rayDir);
            if (std::abs(denom) < 1e-9) {
                // 光线与棋盘格平面平行，保持背景色
                continue;
            }

            double lambda = -D / denom; // 即 N·t / (N·rayDir)
            if (lambda <= 0.0) {
                // 交点在相机后方，保持背景色
                continue;
            }

            // 4. 相机坐标系中的交点
            Eigen::Vector3d P_cam = rayDir * lambda;

            // 5. 变换到棋盘格局部坐标系
            Eigen::Vector3d P_board = R.transpose() * (P_cam - t);

            // 6. 检查是否落在棋盘格范围内
            if (P_board.x() < -renderHalfBw || P_board.x() > renderHalfBw ||
                P_board.y() < -renderHalfBh || P_board.y() > renderHalfBh) {
                continue;
            }

            constexpr uchar kBoardWhite = 180;
            if (P_board.x() < -halfBw || P_board.x() > halfBw ||
                P_board.y() < -halfBh || P_board.y() > halfBh) {
                rowPtr[u] = kBoardWhite;
                continue;
            }

            // 7. 确定该像素对应的方格行列
            int col = static_cast<int>(std::floor((P_board.x() + halfBw) / s));
            int row = static_cast<int>(std::floor((P_board.y() + halfBh) / s));

            // 边界裁切：浮点坐标恰好等于 halfBw 时 col == nSquaresX，需裁到 nSquaresX-1
            col = std::clamp(col, 0, nSquaresX - 1);
            row = std::clamp(row, 0, nSquaresY - 1);

            // Keep white squares below the simulated laser peak so laser-line
            // extraction can reject the board texture with a high threshold.
            uchar color = ((row + col) % 2 == 0) ? 0 : kBoardWhite;
            rowPtr[u] = color;
        }
    }

    return image;
}

// ============================================================
// 激光线渲染
// ============================================================

std::vector<Point3d>
SimulatedScanner::intersectPlaneMesh(const Plane& plane, const TriangleMesh& mesh) {
    std::vector<Point3d> allPts;

    if (mesh.faces.empty()) return allPts;

    // 对每个三角面片计算与平面的交线段
    for (const auto& face : mesh.faces) {
        const Point3d& v0 = mesh.vertices[face[0]];
        const Point3d& v1 = mesh.vertices[face[1]];
        const Point3d& v2 = mesh.vertices[face[2]];

        double d0 = plane.A * v0.x() + plane.B * v0.y() + plane.C * v0.z() + plane.D;
        double d1 = plane.A * v1.x() + plane.B * v1.y() + plane.C * v1.z() + plane.D;
        double d2 = plane.A * v2.x() + plane.B * v2.y() + plane.C * v2.z() + plane.D;

        // 如果三个顶点都在平面同侧（不含平面上的点），无交点
        const double eps = 1e-9;
        if ((d0 >  eps && d1 >  eps && d2 >  eps) ||
            (d0 < -eps && d1 < -eps && d2 < -eps)) {
            continue;
        }

        // 收集边上的交点
        std::vector<Point3d> edgePts;

        auto checkEdge = [&](const Point3d& a, const Point3d& b, double da, double db) {
            if (std::abs(da) < eps && std::abs(db) < eps) {
                // 整条边在平面上，取两个端点
                edgePts.push_back(a);
                edgePts.push_back(b);
            } else if (da * db < 0.0) {
                // 边穿过平面，计算交点
                double t = std::abs(da) / (std::abs(da) + std::abs(db));
                edgePts.push_back(a + t * (b - a));
            } else if (std::abs(da) < eps) {
                edgePts.push_back(a);
            }
            // db == 0 的情况会在对称的调用中处理
        };

        checkEdge(v0, v1, d0, d1);
        checkEdge(v1, v2, d1, d2);
        checkEdge(v2, v0, d2, d0);

        // 去重（合并非常靠近的交点）
        std::vector<Point3d> uniquePoints;
        for (size_t i = 0; i < edgePts.size(); ++i) {
            bool isDuplicate = false;
            for (size_t j = 0; j < i; ++j) {
                if ((edgePts[i] - edgePts[j]).norm() < 1e-4) {
                    isDuplicate = true;
                    break;
                }
            }
            if (!isDuplicate) {
                uniquePoints.push_back(edgePts[i]);
            }
        }

        // 一个三角面与平面的交集至多是一条线段。共面边可能产生
        // 多于两个候选点，此时取距离最远的一对作为真实线段端点。
        if (uniquePoints.size() >= 2) {
            size_t first = 0;
            size_t second = 1;
            double maxDistance = 0.0;
            for (size_t i = 0; i < uniquePoints.size(); ++i) {
                for (size_t j = i + 1; j < uniquePoints.size(); ++j) {
                    const double distance =
                        (uniquePoints[i] - uniquePoints[j]).squaredNorm();
                    if (distance > maxDistance) {
                        maxDistance = distance;
                        first = i;
                        second = j;
                    }
                }
            }
            allPts.push_back(uniquePoints[first]);
            allPts.push_back(uniquePoints[second]);
        }
    }

    return allPts;
}

cv::Mat SimulatedScanner::renderLaserOverlay(const std::vector<Point3d>& pts3d,
                                              int width, int height) {
    cv::Mat overlay(height, width, CV_8UC1, cv::Scalar(0));

    if (pts3d.empty()) return overlay;

    // intersectPlaneMesh 以相邻端点对返回独立线段。逐段绘制才能
    // 保留不同台阶之间的间断，不能跨线段连接成不存在的斜线。
    for (size_t i = 0; i + 1 < pts3d.size(); i += 2) {
        const Point2d first = project(pts3d[i]);
        const Point2d second = project(pts3d[i + 1]);
        const bool firstVisible =
            first.x() >= 0.0 && first.y() >= 0.0 &&
            first.x() < width && first.y() < height;
        const bool secondVisible =
            second.x() >= 0.0 && second.y() >= 0.0 &&
            second.x() < width && second.y() < height;
        if (!firstVisible || !secondVisible) {
            continue;
        }
        cv::line(
            overlay,
            cv::Point(
                static_cast<int>(std::lround(first.x())),
                static_cast<int>(std::lround(first.y()))),
            cv::Point(
                static_cast<int>(std::lround(second.x())),
                static_cast<int>(std::lround(second.y()))),
            cv::Scalar(255), cfg_.laserThickness, cv::LINE_AA);
    }

    // 高斯模糊产生激光线的高斯截面效果
    int ksize = static_cast<int>(std::ceil(cfg_.laserSigma * 6.0));
    if (ksize % 2 == 0) ksize += 1;
    cv::GaussianBlur(overlay, overlay, cv::Size(ksize, ksize),
                     cfg_.laserSigma, cfg_.laserSigma);

    // 缩放到目标亮度
    double maxVal;
    cv::minMaxLoc(overlay, nullptr, &maxVal);
    if (maxVal > 0.0) {
        overlay.convertTo(overlay, CV_8UC1,
                          cfg_.laserIntensity / maxVal);
    }

    return overlay;
}

// ============================================================
// 噪声注入
// ============================================================

void SimulatedScanner::addNoise(cv::Mat& image) {
    // 1. 传感器高斯噪声
    std::normal_distribution<double> sensorDist(0.0, cfg_.sensorNoise);
    cv::Mat noiseImg(image.size(), CV_64FC1);
    for (int r = 0; r < image.rows; ++r) {
        for (int c = 0; c < image.cols; ++c) {
            noiseImg.at<double>(r, c) = sensorDist(rng_);
        }
    }

    cv::Mat floatImg;
    image.convertTo(floatImg, CV_64FC1);
    floatImg += noiseImg;

    // 2. 环境光已在渲染时作为背景加入，此处不再重复

    // 3. 振动噪声：每行独立随机水平偏移
    std::normal_distribution<double> vibDist(0.0, cfg_.vibrationNoise);
    cv::Mat vibResult(image.size(), CV_64FC1);
    for (int r = 0; r < image.rows; ++r) {
        double shift = vibDist(rng_);
        for (int c = 0; c < image.cols; ++c) {
            double srcCol = static_cast<double>(c) - shift;
            int c0 = static_cast<int>(std::floor(srcCol));
            int c1 = c0 + 1;
            double frac = srcCol - static_cast<double>(c0);

            double val = 0.0;
            if (c0 >= 0 && c0 < image.cols)
                val += (1.0 - frac) * floatImg.at<double>(r, c0);
            if (c1 >= 0 && c1 < image.cols)
                val += frac * floatImg.at<double>(r, c1);

            vibResult.at<double>(r, c) = val;
        }
    }

    // 4. 裁切到 [0, 255] 并转回 CV_8U
    cv::Mat clamped;
    vibResult.copyTo(clamped);
    for (int r = 0; r < clamped.rows; ++r) {
        for (int c = 0; c < clamped.cols; ++c) {
            double& v = clamped.at<double>(r, c);
            if (v < 0.0) v = 0.0;
            if (v > 255.0) v = 255.0;
        }
    }
    clamped.convertTo(image, CV_8UC1);
}

// ============================================================
// 随机姿态生成
// ============================================================

Eigen::Isometry3d SimulatedScanner::randomChessboardPose(double nominalZ,
                                                          double maxAngleDeg,
                                                          double maxOffset) {
    std::uniform_real_distribution<double> angleDist(-maxAngleDeg, maxAngleDeg);
    std::uniform_real_distribution<double> offsetDist(-maxOffset, maxOffset);
    std::uniform_real_distribution<double> zDist(nominalZ * 0.8, nominalZ * 1.2);

    // 随机旋转（欧拉角）
    double rx = angleDist(rng_) * M_PI / 180.0;
    double ry = angleDist(rng_) * M_PI / 180.0;
    double rz = angleDist(rng_) * M_PI / 180.0;

    // 随机平移
    double tx = offsetDist(rng_);
    double ty = offsetDist(rng_);
    double tz = zDist(rng_);

    // 构建姿态：先绕 X 轴旋转 180° 使棋盘格面向相机，
    // 再叠加随机小角度扰动，最后平移到指定位置
    //
    // pose * p = R * p + t
    // R = Rx(180°) * Rz(rz) * Ry(ry) * Rx(rx)
    // t = (tx, ty, tz)

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

    Eigen::Matrix3d R =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix() *
        Eigen::AngleAxisd(rz,   Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        Eigen::AngleAxisd(ry,   Eigen::Vector3d::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisd(rx,   Eigen::Vector3d::UnitX()).toRotationMatrix();

    pose.linear() = R;
    pose.translation() = Eigen::Vector3d(tx, ty, tz);

    return pose;
}

bool SimulatedScanner::isChessboardVisible(const Eigen::Isometry3d& pose,
                                            double minVisibleRatio) const {
    std::vector<Point3d> corners3d = getChessboardCorners3d();

    int visibleCount = 0;
    const double margin = 20.0; // 边界容差 (像素)

    for (const auto& c3 : corners3d) {
        Point3d cCam = pose * c3;
        if (cCam.z() <= 0.0) continue; // 在相机后方

        Point2d cImg = project(cCam);
        if (cImg.x() > margin && cImg.x() < cfg_.imgWidth  - margin &&
            cImg.y() > margin && cImg.y() < cfg_.imgHeight - margin) {
            ++visibleCount;
        }
    }

    double ratio = static_cast<double>(visibleCount) / corners3d.size();
    return ratio >= minVisibleRatio;
}

// ============================================================
// 标定图像生成
// ============================================================

std::vector<cv::Mat> SimulatedScanner::generateCalibImages(int numPoses) {
    std::vector<cv::Mat> images;
    images.reserve(numPoses);

    const int maxAttempts = 200;
    int generated = 0;
    int attempt = 0;

    while (generated < numPoses && attempt < maxAttempts) {
        ++attempt;
        // 减小最大角度和偏移，避免棋盘格超出图像边界
        auto pose = randomChessboardPose(cfg_.workingDistance, 6.0, 15.0);

        // 放宽可见性阈值，但棋盘格仍需大部分在画面内
        if (!isChessboardVisible(pose, 1.0)) continue;

        cv::Mat img = renderChessboard(pose);
        addNoise(img);
        images.push_back(img);
        ++generated;
    }

    return images;
}

std::vector<cv::Mat> SimulatedScanner::generateLightPlaneCalibImages(int numPoses) {
    std::vector<cv::Mat> images;
    images.reserve(numPoses);

    // 光平面标定需要棋盘格与光平面相交，才能拍到激光线在棋盘格上的图像。
    // 因此使棋盘格靠近光平面的预期位置：X ≈ 0..30mm, Z ≈ 300mm
    const int maxAttempts = 300;
    int generated = 0;
    int attempt = 0;

    while (generated < numPoses && attempt < maxAttempts) {
        ++attempt;

        // 将棋盘格放置在光平面附近
        std::uniform_real_distribution<double> xOff(-40.0, 60.0);
        std::uniform_real_distribution<double> yOff(-50.0, 50.0);
        std::uniform_real_distribution<double> zDist(cfg_.workingDistance * 0.85,
                                                     cfg_.workingDistance * 1.15);
        std::uniform_real_distribution<double> angleDist(-15.0, 15.0);

        double rx = angleDist(rng_) * M_PI / 180.0;
        double ry = angleDist(rng_) * M_PI / 180.0;
        double rz = angleDist(rng_) * M_PI / 180.0;

        double tx = xOff(rng_);
        double ty = yOff(rng_);
        double tz = zDist(rng_);

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        Eigen::Matrix3d R =
            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(rz,   Eigen::Vector3d::UnitZ()).toRotationMatrix() *
            Eigen::AngleAxisd(ry,   Eigen::Vector3d::UnitY()).toRotationMatrix() *
            Eigen::AngleAxisd(rx,   Eigen::Vector3d::UnitX()).toRotationMatrix();
        pose.linear() = R;
        pose.translation() = Eigen::Vector3d(tx, ty, tz);

        // 确保棋盘格大部分在画面内，提高光平面标定图像质量
        if (!isChessboardVisible(pose, 1.0)) continue;

        // 渲染棋盘格
        cv::Mat img = renderChessboard(pose);

        // 计算棋盘格平面与光平面的交线
        // 棋盘格平面在相机坐标系中由 pose 决定，光平面已知
        // 通过构造一个临时三角网格（两条对角线三角化）来求交线
        double bw = (cfg_.chessboardCols + 1) * cfg_.squareSize;
        double bh = (cfg_.chessboardRows + 1) * cfg_.squareSize;

        TriangleMesh boardMesh;
        // 棋盘格四个角点（在棋盘格坐标系中）
        std::vector<Point3d> boardCornersLocal = {
            {-bw / 2.0, -bh / 2.0, 0.0},
            { bw / 2.0, -bh / 2.0, 0.0},
            { bw / 2.0,  bh / 2.0, 0.0},
            {-bw / 2.0,  bh / 2.0, 0.0}
        };
        for (const auto& bc : boardCornersLocal) {
            boardMesh.vertices.push_back(pose * bc);
        }
        boardMesh.faces = {{0, 1, 2}, {0, 2, 3}};

        // 求光平面与该三角网格的交线
        std::vector<Point3d> laserPts = intersectPlaneMesh(cfg_.lightPlane, boardMesh);

        // 叠加激光线
        if (!laserPts.empty()) {
            cv::Mat laser = renderLaserOverlay(laserPts, cfg_.imgWidth, cfg_.imgHeight);
            cv::add(img, laser, img);
        }

        addNoise(img);
        images.push_back(img);
        ++generated;
    }

    return images;
}

std::vector<cv::Mat> SimulatedScanner::generateMotionAxisCalibImages(int numSteps) {
    std::vector<cv::Mat> images;
    images.reserve(numSteps);

    // Move the board farther from the camera so the complete checkerboard and
    // its backing border stay visible at every axis position.
    const double axisRange =
        cfg_.chessboardRows * cfg_.squareSize * 0.8;
    const double calibrationDistance = cfg_.workingDistance * 1.5;

    // 固定棋盘格面向相机，沿配置的实际移动轴等间距平移。
    for (int i = 0; i < numSteps; ++i) {
        const double axisPosition =
            -axisRange / 2.0 +
            i * axisRange / std::max(numSteps - 1, 1);

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        // 绕 X 轴 180° 使棋盘格面向相机
        pose.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
                            .toRotationMatrix();
        pose.translation() =
            Eigen::Vector3d(0.0, 0.0, calibrationDistance) +
            cfg_.motionAxis.normalized() * axisPosition;

        cv::Mat img = renderChessboard(pose);
        addNoise(img);
        images.push_back(img);
    }

    return images;
}

// ============================================================
// 阶梯块转三角网格
// ============================================================

TriangleMesh SimulatedScanner::stepBlockToMesh(const StepBlock& block,
                                                double workingDistance) {
    TriangleMesh mesh;

    int nSteps = static_cast<int>(block.stepHeights.size());
    if (nSteps == 0) return mesh;

    const double hw = block.width  / 2.0;  // 半宽
    const double hd = block.depth  / 2.0;  // 半深
    const double sw = block.width  / nSteps; // 每级台阶宽度

    // 每级台阶的 X 边界
    std::vector<double> xBounds(nSteps + 1);
    for (int i = 0; i <= nSteps; ++i) {
        xBounds[i] = -hw + i * sw;
    }

    // 添加顶点和面的辅助函数
    auto addVertex = [&](double x, double y, double z) -> int {
        int idx = static_cast<int>(mesh.vertices.size());
        mesh.vertices.emplace_back(x, y, z);
        return idx;
    };

    auto addFace = [&](int a, int b, int c) {
        mesh.faces.push_back({a, b, c});
    };

    auto addQuad = [&](int v00, int v10, int v11, int v01) {
        addFace(v00, v10, v11);
        addFace(v00, v11, v01);
    };

    const double zBase = workingDistance;

    // 为每级台阶构建顶面和垂直面
    struct StepRec {
        double zTop;       // 顶面 Z 坐标
        std::array<int, 4> topVerts;   // 顶面四个顶点索引（顺序：x0y0, x1y0, x1y1, x0y1）
        std::array<int, 4> baseVerts;  // 底面四个顶点索引
    };

    std::vector<StepRec> steps(nSteps);
    for (int i = 0; i < nSteps; ++i) {
        double x0 = xBounds[i];
        double x1 = xBounds[i + 1];
        double zTop = zBase - block.stepHeights[i];
        double y0 = -hd;
        double y1 =  hd;

        StepRec& sr = steps[i];
        sr.zTop = zTop;
        sr.topVerts[0] = addVertex(x0, y0, zTop);
        sr.topVerts[1] = addVertex(x1, y0, zTop);
        sr.topVerts[2] = addVertex(x1, y1, zTop);
        sr.topVerts[3] = addVertex(x0, y1, zTop);
        sr.baseVerts[0] = addVertex(x0, y0, zBase);
        sr.baseVerts[1] = addVertex(x1, y0, zBase);
        sr.baseVerts[2] = addVertex(x1, y1, zBase);
        sr.baseVerts[3] = addVertex(x0, y1, zBase);
    }

    // 生成面
    for (int i = 0; i < nSteps; ++i) {
        const StepRec& sr = steps[i];

        // 顶面（最重要的面，激光线会打到上面）
        addQuad(sr.topVerts[0], sr.topVerts[1], sr.topVerts[2], sr.topVerts[3]);

        // 前面（Y = -hd）
        addQuad(sr.topVerts[0], sr.topVerts[1], sr.baseVerts[1], sr.baseVerts[0]);

        // 后面（Y = +hd）
        addQuad(sr.topVerts[3], sr.topVerts[2], sr.baseVerts[2], sr.baseVerts[3]);

        // 左面（X = x0，最左侧台阶）或台阶过渡面
        if (i == 0) {
            addQuad(sr.topVerts[0], sr.topVerts[3], sr.baseVerts[3], sr.baseVerts[0]);
        } else {
            // 与左侧台阶之间的过渡垂直面
            const StepRec& sl = steps[i - 1];
            addQuad(sl.topVerts[1],  // 左台阶右前上
                    sr.topVerts[0],   // 右台阶左前上
                    sr.baseVerts[0],  // 右台阶左前下
                    sl.baseVerts[1]); // 左台阶右前下
            addQuad(sl.topVerts[2],  // 左台阶右后上
                    sr.topVerts[3],   // 右台阶左后上
                    sr.baseVerts[3],  // 右台阶左后下
                    sl.baseVerts[2]); // 左台阶右后下
        }

        // 右面（最右侧台阶）或过渡面已在下一级台阶的左面处理
        if (i == nSteps - 1) {
            addQuad(sr.topVerts[1], sr.topVerts[2], sr.baseVerts[2], sr.baseVerts[1]);
        }
    }

    return mesh;
}

// ============================================================
// 物体扫描
// ============================================================

std::pair<std::vector<cv::Mat>, PointCloud>
SimulatedScanner::generateScan(const TriangleMesh& object) {
    std::vector<cv::Mat> images;
    PointCloud gtCloud;

    int numSteps = static_cast<int>(cfg_.travelRange / cfg_.defaultStep) + 1;

    for (int i = 0; i < numSteps; ++i) {
        const double motionPos = scanPosition(static_cast<size_t>(i));

        // 扫描器的重建约定是“测得截面 + 编码器位移 = 工件坐标”。
        // 因此仿真渲染时让工件沿移动轴反向运动，重建时再加回编码器
        // 位移，最终恢复到工件自身坐标系，避免同一位移被重复累计。
        TriangleMesh movedMesh = object;
        const Eigen::Vector3d offset =
            -cfg_.motionAxis.normalized() * motionPos;
        for (auto& v : movedMesh.vertices) {
            v += offset;
        }

        // 计算光平面与移动后物体的交线（ground truth 3D 点）
        const std::vector<Point3d> cameraIntersection =
            intersectPlaneMesh(cfg_.lightPlane, movedMesh);

        // GT 也转换回工件自身坐标系，才能与重建结果逐点比较。
        for (const auto& point : cameraIntersection) {
            gtCloud.push_back(point - offset);
        }

        // 生成背景图像
        cv::Mat image(cfg_.imgHeight, cfg_.imgWidth, CV_8UC1,
                      cv::Scalar(static_cast<int>(cfg_.ambientLight)));

        // 渲染激光线
        cv::Mat laser = renderLaserOverlay(
            cameraIntersection, cfg_.imgWidth, cfg_.imgHeight);
        cv::add(image, laser, image);

        // 注入噪声
        addNoise(image);

        images.push_back(image);
    }

    return {images, gtCloud};
}

std::pair<std::vector<cv::Mat>, PointCloud>
SimulatedScanner::generateScan(const StepBlock& block) {
    TriangleMesh mesh = stepBlockToMesh(block, cfg_.workingDistance);
    return generateScan(mesh);
}

} // namespace sim
} // namespace lsc
