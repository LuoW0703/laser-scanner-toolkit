#include "lsc/calib/light_plane_calib.h"
#include "lsc/core/log.h"

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace {

bool detectChessboard(const cv::Mat& gray, const cv::Size& boardSize,
                      std::vector<cv::Point2f>& corners) {
    const double scale = std::min(1.0, 800.0 / static_cast<double>(gray.cols));
    cv::Mat detectionImage;
    if (scale < 1.0) {
        cv::resize(gray, detectionImage, cv::Size(), scale, scale, cv::INTER_AREA);
    } else {
        detectionImage = gray;
    }

    const int flags =
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    if (!cv::findChessboardCorners(
            detectionImage, boardSize, corners, flags)) {
        corners.clear();
        if (!cv::findChessboardCornersSB(
                detectionImage, boardSize, corners,
                cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE)) {
            return false;
        }
    }
    if (scale < 1.0) {
        for (auto& corner : corners) {
            corner.x = static_cast<float>(corner.x / scale);
            corner.y = static_cast<float>(corner.y / scale);
        }
    }
    cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(
            cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    return true;
}

} // namespace

namespace lsc {

bool LightPlaneCalibrator::calibrate(
    const std::vector<cv::Mat>& images,
    const CameraIntrinsics& K,
    const cv::Size& boardSize,
    double squareSize,
    Plane& outPlane,
    double& outRmsError,
    LaserMethod laserMethod,
    double laserThresh,
    double laserSigma) {
    if (images.empty()) {
        LSC_ERROR("LightPlaneCalib") << "calibrate: no input images";
        return false;
    }
    if (boardSize.width < 2 || boardSize.height < 2 || squareSize <= 0.0) {
        LSC_ERROR("LightPlaneCalib")
            << "calibrate: invalid board geometry";
        return false;
    }

    // 每张标定图提供“激光线 ∩ 当前标定板平面”的一组 3D 点。
    // 标定板姿态必须变化：单张图中的交点近似共线，多姿态交线共同
    // 覆盖光平面后，才能稳定约束二维平面参数。
    std::vector<Point3d> allPoints3D;
    size_t validImageCount = 0;
    for (size_t i = 0; i < images.size(); ++i) {
        const auto points = computeLaser3DPoints(
            images[i], K, boardSize, squareSize,
            laserMethod, laserThresh, laserSigma);
        if (points.empty()) {
            LSC_WARN("LightPlaneCalib")
                << "calibrate: image " << i
                << " has no valid laser 3D points, skipping";
            continue;
        }
        allPoints3D.insert(
            allPoints3D.end(), points.begin(), points.end());
        ++validImageCount;
    }

    if (allPoints3D.size() < 3 || validImageCount < 2) {
        LSC_ERROR("LightPlaneCalib")
            << "calibrate: valid images=" << validImageCount
            << ", valid 3D points=" << allPoints3D.size();
        return false;
    }

    LSC_INFO("LightPlaneCalib")
        << "calibrate: valid images=" << validImageCount
        << ", total points=" << allPoints3D.size();
    if (!fitPlaneSVD(allPoints3D, outPlane, outRmsError)) {
        return false;
    }

    LSC_INFO("LightPlaneCalib")
        << "calibrate: rms=" << outRmsError << " mm";
    return true;
}

bool LightPlaneCalibrator::fitPlaneSVD(
    const std::vector<Point3d>& points,
    Plane& outPlane,
    double& outResidual) {
    if (points.size() < 3) {
        LSC_ERROR("LightPlaneCalib")
            << "fitPlaneSVD: points=" << points.size()
            << ", need at least 3";
        return false;
    }

    // 对点集中心化后，最小奇异值对应的右奇异向量是最小二乘平面
    // 法向量。中心化同时改善数值条件，并使 D=-n·centroid 可直接求得。
    Point3d centroid = Point3d::Zero();
    for (const auto& point : points) {
        centroid += point;
    }
    centroid /= static_cast<double>(points.size());

    Eigen::MatrixXd centered(points.size(), 3);
    for (size_t i = 0; i < points.size(); ++i) {
        centered.row(i) = (points[i] - centroid).transpose();
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        centered, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::Vector3d normal = svd.matrixV().col(2);

    outPlane.A = normal.x();
    outPlane.B = normal.y();
    outPlane.C = normal.z();
    outPlane.D = -normal.dot(centroid);
    outPlane.normalize();

    const Eigen::Vector3d normalizedNormal(
        outPlane.A, outPlane.B, outPlane.C);
    double squaredError = 0.0;
    for (const auto& point : points) {
        const double distance =
            normalizedNormal.dot(point) + outPlane.D;
        squaredError += distance * distance;
    }
    outResidual =
        std::sqrt(squaredError / static_cast<double>(points.size()));
    return true;
}

std::vector<Point3d> LightPlaneCalibrator::computeLaser3DPoints(
    const cv::Mat& image, const CameraIntrinsics& K,
    const cv::Size& boardSize, double squareSize,
    LaserMethod method, double thresh, double sigma) {
    std::vector<Point3d> result;
    if (image.empty()) {
        return result;
    }

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    // 棋盘角点建立“已知板坐标 -> 图像像素”的对应关系，solvePnP
    // 据此恢复标定板在相机坐标系中的刚体位姿。
    std::vector<cv::Point2f> corners;
    if (!detectChessboard(gray, boardSize, corners)) {
        LSC_WARN("LightPlaneCalib")
            << "computeLaser3DPoints: chessboard detection failed";
        return result;
    }

    std::vector<cv::Point3f> objectPoints;
    objectPoints.reserve(boardSize.width * boardSize.height);
    for (int row = 0; row < boardSize.height; ++row) {
        for (int column = 0; column < boardSize.width; ++column) {
            objectPoints.emplace_back(
                static_cast<float>(column * squareSize),
                static_cast<float>(row * squareSize),
                0.0f);
        }
    }

    cv::Mat rotationVector;
    cv::Mat translationVector;
    if (!cv::solvePnP(
            objectPoints, corners, K.matrix33(), K.distCoeffs(),
            rotationVector, translationVector)) {
        LSC_WARN("LightPlaneCalib")
            << "computeLaser3DPoints: solvePnP failed";
        return result;
    }

    // 激光中心提取与棋盘检测相互独立：棋盘负责给出承载平面，
    // 激光中心负责给出待求交的相机射线。
    LaserLineExtractor extractor;
    const auto laserPoints =
        extractor.extract(gray, method, thresh, sigma);
    if (laserPoints.size() < 3) {
        LSC_WARN("LightPlaneCalib")
            << "computeLaser3DPoints: too few laser points: "
            << laserPoints.size();
        return result;
    }

    // 棋盘局部 Z 轴经过旋转后即为相机坐标系中的板平面法向；
    // translationVector 是棋盘局部原点，因此平面满足
    //   n_board · (P - boardOrigin) = 0。
    cv::Mat rotationMatrix;
    cv::Rodrigues(rotationVector, rotationMatrix);
    Eigen::Matrix3d boardRotation;
    Eigen::Vector3d boardOrigin;
    cv::cv2eigen(rotationMatrix, boardRotation);
    cv::cv2eigen(translationVector, boardOrigin);
    const Eigen::Vector3d boardNormal = boardRotation.col(2);

    // 像素先去畸变并归一化，normalized=(x,y) 对应射线 (x,y,1)。
    // 不能直接用原像素减主点，因为径向/切向畸变会系统性弯曲激光线。
    std::vector<cv::Point2f> distortedPoints;
    distortedPoints.reserve(laserPoints.size());
    for (const auto& point : laserPoints) {
        distortedPoints.emplace_back(
            static_cast<float>(point.x()),
            static_cast<float>(point.y()));
    }
    std::vector<cv::Point2f> normalizedPoints;
    cv::undistortPoints(
        distortedPoints, normalizedPoints,
        K.matrix33(), K.distCoeffs());

    const double minX = -squareSize;
    const double maxX = boardSize.width * squareSize;
    const double minY = -squareSize;
    const double maxY = boardSize.height * squareSize;
    result.reserve(normalizedPoints.size());

    for (const auto& normalized : normalizedPoints) {
        Eigen::Vector3d ray(normalized.x, normalized.y, 1.0);
        const double denominator = boardNormal.dot(ray);
        if (std::abs(denominator) < 1e-12) {
            continue;
        }

        // 射线 P=t*r 代入板平面：
        // n·(t*r-origin)=0 => t=n·origin/(n·r)。
        const double distance = boardNormal.dot(boardOrigin) / denominator;
        if (distance <= 0.0) {
            continue;
        }

        const Point3d pointInCamera = ray * distance;
        // 把交点反变换回棋盘局部坐标，仅保留落在实体标定板附近的点，
        // 防止背景高亮结构被误识别为激光后参与光平面拟合。
        const Point3d pointOnBoard =
            boardRotation.transpose() * (pointInCamera - boardOrigin);
        if (pointOnBoard.x() < minX || pointOnBoard.x() > maxX ||
            pointOnBoard.y() < minY || pointOnBoard.y() > maxY) {
            continue;
        }
        result.push_back(pointInCamera);
    }

    return result;
}

} // namespace lsc
