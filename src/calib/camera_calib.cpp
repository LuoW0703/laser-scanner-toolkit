#include "lsc/calib/camera_calib.h"
#include "lsc/core/log.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/persistence.hpp>
#include <algorithm>
#include <cmath>

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

    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    if (!cv::findChessboardCorners(detectionImage, boardSize, corners, flags)) {
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
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    return true;
}

} // namespace

namespace lsc {

bool CameraCalibrator::calibrate(
    const std::vector<std::string>& imagePaths,
    const cv::Size& boardSize,
    double squareSize,
    CameraIntrinsics& outIntrinsics,
    double& outRmsError) {
    std::vector<cv::Mat> images;
    images.reserve(imagePaths.size());
    for (const auto& path : imagePaths) {
        cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            LSC_WARN("CameraCalib") << "calibrate: cannot read image, skipping: " << path;
            continue;
        }
        images.push_back(img);
    }
    return calibrateFromImages(images, boardSize, squareSize, outIntrinsics, outRmsError);
}

bool CameraCalibrator::calibrateFromImages(
    const std::vector<cv::Mat>& images,
    const cv::Size& boardSize,
    double squareSize,
    CameraIntrinsics& outIntrinsics,
    double& outRmsError) {
    m_perViewErrors.clear();

    if (images.empty()) {
        LSC_ERROR("CameraCalib") << "calibrateFromImages: no input images";
        return false;
    }

    // 棋盘格局部坐标系定义在标定板平面上，Z 恒为 0。每张图都复用
    // 同一组已知 3D 点，而检测到的 2D 角点随标定板姿态变化。
    // 多姿态提供了焦距、主点、畸变和外参之间的独立约束。
    std::vector<cv::Point3f> objectPoints;
    objectPoints.reserve(boardSize.width * boardSize.height);
    for (int r = 0; r < boardSize.height; ++r) {
        for (int c = 0; c < boardSize.width; ++c) {
            objectPoints.push_back(cv::Point3f(
                static_cast<float>(c * squareSize),
                static_cast<float>(r * squareSize),
                0.0f));
        }
    }

    std::vector<std::vector<cv::Point3f>> allObjectPoints;
    std::vector<std::vector<cv::Point2f>> allImagePoints;
    cv::Size imageSize;

    for (size_t idx = 0; idx < images.size(); ++idx) {
        const cv::Mat& img = images[idx];
        if (img.empty()) {
            LSC_WARN("CameraCalib")
                << "calibrateFromImages: image " << idx
                << " is empty, skipping";
            continue;
        }
        if (imageSize.empty()) {
            imageSize = img.size();
        } else if (img.size() != imageSize) {
            LSC_WARN("CameraCalib")
                << "calibrateFromImages: image " << idx
                << " has a different size, skipping";
            continue;
        }

        // 棋盘检测使用灰度图。
        cv::Mat gray;
        if (img.channels() == 3) {
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = img;
        }

        std::vector<cv::Point2f> corners;
        bool found = detectChessboard(gray, boardSize, corners);

        if (!found) {
            LSC_WARN("CameraCalib") << "calibrateFromImages: chessboard detection failed for image " << idx << ", skipping";
            continue;
        }

        allObjectPoints.push_back(objectPoints);
        allImagePoints.push_back(corners);
    }

    if (allObjectPoints.size() < 3) {
        LSC_ERROR("CameraCalib") << "calibrateFromImages: detected chessboards=" << allObjectPoints.size() << ", need at least 3";
        return false;
    }

    LSC_INFO("CameraCalib") << "calibrateFromImages: detected chessboards=" << allObjectPoints.size();

    // calibrateCamera 最小化所有视图的重投影平方误差：
    //   observed_ij - project(K, distortion, R_i, t_i, object_j)
    // 输出共享内参 K/畸变，以及每张图独立的外参 R_i,t_i。
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(
        allObjectPoints, allImagePoints, imageSize,
        cameraMatrix, distCoeffs, rvecs, tvecs);

    outIntrinsics = CameraIntrinsics::fromOpenCV(
        cameraMatrix, distCoeffs, imageSize.width, imageSize.height);
    outRmsError = rms;

    // 全局 RMS 适合总体质量门限；逐视图 RMS 可以定位模糊、遮挡或
    // 姿态退化的单张图片，便于在生产标定中有针对性地剔除后重算。
    m_perViewErrors.clear();
    for (size_t i = 0; i < allObjectPoints.size(); ++i) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(allObjectPoints[i], rvecs[i], tvecs[i],
                          cameraMatrix, distCoeffs, projected);

        double viewErr = 0.0;
        for (size_t j = 0; j < allImagePoints[i].size(); ++j) {
            double dx = allImagePoints[i][j].x - projected[j].x;
            double dy = allImagePoints[i][j].y - projected[j].y;
            viewErr += dx * dx + dy * dy;
        }
        viewErr = std::sqrt(viewErr / allImagePoints[i].size());
        m_perViewErrors.push_back(viewErr);
    }

    LSC_INFO("CameraCalib") << "calibrateFromImages: rms=" << rms << " px";

    return true;
}

std::vector<double> CameraCalibrator::perViewErrors() const {
    return m_perViewErrors;
}

bool CameraCalibrator::save(const std::string& yamlPath,
                            const CameraIntrinsics& K) const {
    cv::FileStorage fs(yamlPath, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        LSC_ERROR("CameraCalib") << "save: cannot open file for writing: " << yamlPath;
        return false;
    }
    fs << "fx" << K.fx;
    fs << "fy" << K.fy;
    fs << "cx" << K.cx;
    fs << "cy" << K.cy;
    fs << "k1" << K.k1;
    fs << "k2" << K.k2;
    fs << "k3" << K.k3;
    fs << "p1" << K.p1;
    fs << "p2" << K.p2;
    fs << "imgWidth" << K.imgWidth;
    fs << "imgHeight" << K.imgHeight;
    fs.release();
    return true;
}

bool CameraCalibrator::load(const std::string& yamlPath,
                            CameraIntrinsics& K) const {
    cv::FileStorage fs(yamlPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        LSC_ERROR("CameraCalib") << "load: cannot open file for reading: " << yamlPath;
        return false;
    }
    fs["fx"] >> K.fx;
    fs["fy"] >> K.fy;
    fs["cx"] >> K.cx;
    fs["cy"] >> K.cy;
    fs["k1"] >> K.k1;
    fs["k2"] >> K.k2;
    fs["k3"] >> K.k3;
    fs["p1"] >> K.p1;
    fs["p2"] >> K.p2;
    fs["imgWidth"] >> K.imgWidth;
    fs["imgHeight"] >> K.imgHeight;
    fs.release();
    return true;
}

} // namespace lsc
