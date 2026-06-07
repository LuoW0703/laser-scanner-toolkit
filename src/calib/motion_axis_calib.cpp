#include "lsc/calib/motion_axis_calib.h"
#include "lsc/core/log.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>
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

bool MotionAxisCalibrator::computeBoardOrigin(
    const cv::Mat& image, const CameraIntrinsics& K,
    const cv::Size& boardSize, double squareSize,
    Eigen::Vector3d& outOrigin) {
    if (image.empty()) {
        LSC_WARN("MotionAxisCalib")
            << "computeBoardOrigin: input image is empty";
        return false;
    }

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    // 角点提供标定板局部坐标和图像坐标的对应关系。相机内参已知后，
    // solvePnP 可恢复当前帧标定板的旋转和平移。
    std::vector<cv::Point2f> corners;
    if (!detectChessboard(gray, boardSize, corners)) {
        LSC_WARN("MotionAxisCalib")
            << "computeBoardOrigin: chessboard detection failed";
        return false;
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
        LSC_WARN("MotionAxisCalib")
            << "computeBoardOrigin: solvePnP failed";
        return false;
    }

    // 使用棋盘中心而不是第一个角点作为轨迹点。部分棋盘姿态下 OpenCV
    // 可能反转角点遍历方向，中心点在这种顺序变化下保持不变。
    cv::Mat rotation;
    cv::Rodrigues(rotationVector, rotation);
    const cv::Mat boardCenter = (cv::Mat_<double>(3, 1)
        << (boardSize.width - 1) * squareSize * 0.5,
           (boardSize.height - 1) * squareSize * 0.5,
           0.0);
    const cv::Mat centerInCamera =
        rotation * boardCenter + translationVector;
    outOrigin.x() = centerInCamera.at<double>(0);
    outOrigin.y() = centerInCamera.at<double>(1);
    outOrigin.z() = centerInCamera.at<double>(2);
    return true;
}

bool MotionAxisCalibrator::calibrate(
    const std::vector<cv::Mat>& images,
    const CameraIntrinsics& K,
    const cv::Size& boardSize,
    double squareSize,
    const std::vector<double>& positions,
    Eigen::Vector3d& outAxis,
    double& outRmsError) {
    if (images.empty()) {
        LSC_ERROR("MotionAxisCalib") << "calibrate: no input images";
        return false;
    }
    if (!positions.empty() && positions.size() != images.size()) {
        LSC_ERROR("MotionAxisCalib")
            << "calibrate: positions size=" << positions.size()
            << " does not match images size=" << images.size();
        return false;
    }

    // origins 与 validPositions 必须同步压入。若中间某张图检测失败，
    // 仍要保持每个三维中心与原始位置指令一一对应，不能按紧凑下标
    // 重新读取 positions，否则方向符号会被错误校正。
    std::vector<Eigen::Vector3d> origins;
    std::vector<double> validPositions;
    origins.reserve(images.size());
    validPositions.reserve(images.size());

    for (size_t i = 0; i < images.size(); ++i) {
        Eigen::Vector3d origin;
        if (computeBoardOrigin(
                images[i], K, boardSize, squareSize, origin)) {
            origins.push_back(origin);
            if (!positions.empty()) {
                validPositions.push_back(positions[i]);
            }
        } else {
            LSC_WARN("MotionAxisCalib")
                << "calibrate: image " << i
                << " chessboard detection failed, skipping";
        }
    }

    if (origins.size() < 2) {
        LSC_ERROR("MotionAxisCalib")
            << "calibrate: valid origins=" << origins.size()
            << ", need at least 2";
        return false;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& origin : origins) {
        centroid += origin;
    }
    centroid /= static_cast<double>(origins.size());

    if (origins.size() == 2) {
        const Eigen::Vector3d direction = origins[1] - origins[0];
        const double length = direction.norm();
        if (length < 1e-12) {
            LSC_ERROR("MotionAxisCalib")
                << "calibrate: two origins are coincident";
            return false;
        }
        outAxis = direction / length;
    } else {
        Eigen::MatrixXd centered(origins.size(), 3);
        for (size_t i = 0; i < origins.size(); ++i) {
            centered.row(i) = (origins[i] - centroid).transpose();
        }

        // PCA 直线拟合：协方差最大特征值方向表示样本方差最大的方向，
        // 即标定板中心随移动机构运动形成的主轴方向。
        Eigen::Matrix3d covariance = centered.transpose() * centered;
        covariance /= static_cast<double>(origins.size() - 1);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() != Eigen::Success) {
            LSC_ERROR("MotionAxisCalib")
                << "calibrate: eigen decomposition failed";
            return false;
        }
        outAxis = solver.eigenvectors().col(2);
    }

    // PCA 只确定一条无向直线，axis 和 -axis 等价。将点在轴上的投影
    // 与编码器/指令位置做相关性判断，使最终方向与位置递增方向一致。
    if (!validPositions.empty()) {
        double meanPosition = 0.0;
        for (const double position : validPositions) {
            meanPosition += position;
        }
        meanPosition /= static_cast<double>(validPositions.size());

        double signCheck = 0.0;
        for (size_t i = 0; i < origins.size(); ++i) {
            signCheck += outAxis.dot(origins[i] - centroid) *
                         (validPositions[i] - meanPosition);
        }
        if (signCheck < 0.0) {
            outAxis = -outAxis;
        }
    }

    // 点到拟合轴的正交距离反映机构直线度、角点定位和 PnP 噪声的综合
    // 影响。这里报告 RMS，便于跨不同采样数量比较标定质量。
    outRmsError = 0.0;
    for (const auto& origin : origins) {
        const Eigen::Vector3d offset = origin - centroid;
        const double distance =
            (offset - outAxis * offset.dot(outAxis)).norm();
        outRmsError += distance * distance;
    }
    outRmsError = std::sqrt(outRmsError / origins.size());

    LSC_INFO("MotionAxisCalib")
        << "calibrate: axis=(" << outAxis.x() << ", "
        << outAxis.y() << ", " << outAxis.z()
        << "), rms=" << outRmsError << " mm";
    return true;
}

} // namespace lsc
