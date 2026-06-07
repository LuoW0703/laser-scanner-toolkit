#pragma once
#include "lsc/core/types.h"
#include <vector>
#include <string>

namespace lsc {

/**
 * 相机内参标定器（张正友标定法）
 *
 * 使用棋盘格标定板，通过多视角图像估计相机内参和畸变系数。
 * 内部封装 OpenCV 的 calibrateCamera 流程。
 */
class CameraCalibrator {
public:
    /**
     * 从图像文件进行标定
     * @param imagePaths    标定图像文件路径列表（至少 10 张不同姿态的图像）
     * @param boardSize     棋盘格内角点数 (cols, rows)
     * @param squareSize    棋盘格方格尺寸 (mm)
     * @param outIntrinsics 输出相机内参
     * @param outRmsError   输出重投影均方根误差 (px)
     * @return              true=标定成功
     */
    bool calibrate(
        const std::vector<std::string>& imagePaths,
        const cv::Size& boardSize,
        double squareSize,
        CameraIntrinsics& outIntrinsics,
        double& outRmsError
    );

    /**
     * 从内存中的图像进行标定
     */
    bool calibrateFromImages(
        const std::vector<cv::Mat>& images,
        const cv::Size& boardSize,
        double squareSize,
        CameraIntrinsics& outIntrinsics,
        double& outRmsError
    );

    /**
     * 返回每张图像的重投影误差，调用 calibrate 之后有效
     */
    std::vector<double> perViewErrors() const;

    /**
     * 将相机内参加保存为 YAML 文件
     */
    bool save(const std::string& yamlPath, const CameraIntrinsics& K) const;

    /**
     * 从 YAML 文件加载相机内参
     */
    bool load(const std::string& yamlPath, CameraIntrinsics& K) const;

private:
    std::vector<double> m_perViewErrors;
};

} // namespace lsc
