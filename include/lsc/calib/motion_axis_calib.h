#pragma once
#include "lsc/core/types.h"

namespace lsc {

/**
 * 移动轴标定器
 *
 * 通过将标定板沿移动轴平移到不同位置，拍照并检测标定板原点，
 * 拟合出移动轴的方向向量（在相机坐标系中）。
 *
 * 流程：
 * 1. 在移动轴不同位置（已知位置值或多组相对位置）拍摄标定板图像
 * 2. 用已标定的相机内参估计每个位置的标定板位姿，提取棋盘格原点
 * 3. 通过线性拟合得到移动轴方向
 */
class MotionAxisCalibrator {
public:
    /**
     * 标定移动轴方向
     *
     * @param images       不同移动轴位置的标定板图像
     * @param K            已标定的相机内参
     * @param boardSize    棋盘格内角点数
     * @param squareSize   棋盘格方格尺寸 (mm)
     * @param positions    每张图像对应的移动轴位置 (mm)。可为空；非空时长度须与 images 一致
     * @param outAxis      输出移动轴单位方向向量（相机坐标系）
     * @param outRmsError  输出拟合残差 (mm)
     * @return             true=标定成功
     */
    bool calibrate(
        const std::vector<cv::Mat>& images,
        const CameraIntrinsics& K,
        const cv::Size& boardSize,
        double squareSize,
        const std::vector<double>& positions,
        Eigen::Vector3d& outAxis,
        double& outRmsError
    );

private:
    /**
     * 从单张棋盘格图像计算标定板中心在相机坐标系中的 3D 坐标。
     * 使用中心点可避免棋盘角点顺序在相邻图像中反转时引入跳变。
     */
    bool computeBoardOrigin(const cv::Mat& image, const CameraIntrinsics& K,
                            const cv::Size& boardSize, double squareSize,
                            Eigen::Vector3d& outOrigin);
};

} // namespace lsc
