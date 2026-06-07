#pragma once
#include "lsc/core/types.h"
#include "lsc/core/laser_line.h"

namespace lsc {

/**
 * 光平面标定器
 *
 * 利用已知的相机内参和棋盘格标定板，通过提取激光线在标定板上的投影点，
 * 将多组激光线点从图像坐标系转换到相机 3D 坐标系，
 * 最后通过 SVD 拟合平面得到光平面方程。
 */
class LightPlaneCalibrator {
public:
    /**
     * 标定光平面
     *
     * @param images       包含激光线的标定板图像（激光应照射在棋盘格上）
     * @param K            已标定的相机内参
     * @param boardSize    棋盘格内角点数
     * @param squareSize   棋盘格方格尺寸 (mm)
     * @param outPlane     输出光平面参数 (A,B,C,D)
     * @param outRmsError  输出平面拟合残差 (mm)
     * @param laserMethod  激光线提取方法
     * @param laserThresh  激光线提取阈值
     * @param laserSigma   激光线提取 sigma（Steger 方法用）
     * @return             true=标定成功
     */
    bool calibrate(
        const std::vector<cv::Mat>& images,
        const CameraIntrinsics& K,
        const cv::Size& boardSize,
        double squareSize,
        Plane& outPlane,
        double& outRmsError,
        LaserMethod laserMethod = LaserMethod::GRAY_CENTROID,
        double laserThresh = 50.0,
        double laserSigma = 1.5
    );

    /**
     * SVD 平面拟合（静态方法，可独立使用）
     *
     * 对一组 3D 点进行最小二乘平面拟合。
     * @param points      输入 3D 点集
     * @param outPlane    输出平面参数
     * @param outResidual 输出拟合残差 (mm)
     * @return            true=拟合成功
     */
    static bool fitPlaneSVD(const std::vector<Point3d>& points,
                            Plane& outPlane, double& outResidual);

private:
    /**
     * 计算单张图像中激光线在标定板上的 3D 坐标
     *
     * 流程：
     * 1. 检测棋盘角点并通过 solvePnP 估计当前标定板平面
     * 2. 提取激光线亚像素坐标并去除相机畸变
     * 3. 构造每个激光像素的相机射线，与当前标定板平面求交
     * 4. 过滤落在实体标定板范围外的误检测点
     */
    std::vector<Point3d> computeLaser3DPoints(
        const cv::Mat& image, const CameraIntrinsics& K,
        const cv::Size& boardSize, double squareSize,
        LaserMethod method, double thresh, double sigma);
};

} // namespace lsc
