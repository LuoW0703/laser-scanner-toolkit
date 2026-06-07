#pragma once
#include "lsc/core/types.h"
#include <opencv2/core.hpp>

namespace lsc {

/** 激光线提取方法枚举 */
enum class LaserMethod {
    GRAY_CENTROID,  // 灰度重心法（速度快，默认推荐）
    STEGER          // Steger 海森矩阵法（精度高，可选）
};

/**
 * 从灰度图像中提取激光线中心（亚像素精度）
 *
 * 假设激光线大致水平（横跨图像宽度），逐列扫描提取每列的亚像素行坐标。
 * 返回 N 个 (col, row_subpixel) 点，点数可能少于图像宽度（低于阈值的列会被丢弃）。
 */
class LaserLineExtractor {
public:
    /**
     * 提取整幅图像中的激光线中心
     * @param grayImage  输入灰度图像（单通道 CV_8U 或 CV_32F）
     * @param method     提取方法，默认灰度重心法
     * @param threshold  灰度阈值，低于此值的像素不参与计算（GRAY_CENTROID 默认 50）
     * @param sigma      Steger 法中高斯平滑的标准差（仅 STEGER 方法使用，默认 1.5）
     * @return          亚像素激光线中心点集合（图像坐标）
     */
    std::vector<Point2d> extract(
        const cv::Mat& grayImage,
        LaserMethod method = LaserMethod::GRAY_CENTROID,
        double threshold = 50.0,
        double sigma = 1.5
    );

private:
    std::vector<Point2d> extractGrayCentroid(const cv::Mat& image, double threshold);
    std::vector<Point2d> extractSteger(const cv::Mat& image, double sigma, double threshold);
};

} // namespace lsc
