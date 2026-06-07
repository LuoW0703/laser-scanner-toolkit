#include "lsc/core/laser_line.h"
#include "lsc/core/log.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>

namespace lsc {

std::vector<Point2d> LaserLineExtractor::extract(
    const cv::Mat& grayImage,
    LaserMethod method,
    double threshold,
    double sigma) {
    if (method == LaserMethod::STEGER) {
        return extractSteger(grayImage, sigma, threshold);
    }
    return extractGrayCentroid(grayImage, threshold);
}

std::vector<Point2d> LaserLineExtractor::extractGrayCentroid(
    const cv::Mat& image, double threshold) {
    std::vector<Point2d> result;

    // 灰度重心法要求单通道图像。
    if (image.channels() != 1) {
        LSC_WARN("LaserLine") << "extractGrayCentroid: input image is not grayscale";
        return result;
    }

    const int cols = image.cols;
    const int rows = image.rows;

    // 逐列扫描，假设激光线大致沿图像水平方向延伸。
    for (int col = 0; col < cols; ++col) {
        // 记录当前列中所有连续超过阈值的像素区间。
        struct Interval {
            int startRow, endRow;
            double peakVal;
            double centroidRow;
        };
        std::vector<Interval> intervals;

        bool inInterval = false;
        int startRow = 0;
        double maxVal = 0.0;
        double sumWeighted = 0.0;  // 危(row * gray)
        double sumWeight   = 0.0;  // 危(gray)

        // 从上到下扫描当前列。
        for (int row = 0; row < rows; ++row) {
            double val = 0.0;
            if (image.type() == CV_8U) {
                val = static_cast<double>(image.at<uint8_t>(row, col));
            } else if (image.type() == CV_32F) {
                val = static_cast<double>(image.at<float>(row, col));
            }

            if (val > threshold) {
                if (!inInterval) {
                    inInterval = true;
                    startRow = row;
                    maxVal = val;
                    sumWeighted = static_cast<double>(row) * val;
                    sumWeight = val;
                } else {
                    // 延续当前高亮区间。
                    if (val > maxVal) maxVal = val;
                    sumWeighted += static_cast<double>(row) * val;
                    sumWeight += val;
                }
            } else {
                if (inInterval) {
                    Interval iv;
                    iv.startRow = startRow;
                    iv.endRow = row - 1;
                    iv.peakVal = maxVal;
                    iv.centroidRow = (sumWeight > 1e-12) ? (sumWeighted / sumWeight) : 0.0;
                    intervals.push_back(iv);
                    inInterval = false;
                }
            }
        }
        // 处理延伸到图像底部的区间。
        if (inInterval) {
            Interval iv;
            iv.startRow = startRow;
            iv.endRow = rows - 1;
            iv.peakVal = maxVal;
            iv.centroidRow = (sumWeight > 1e-12) ? (sumWeighted / sumWeight) : 0.0;
            intervals.push_back(iv);
        }

        if (!intervals.empty()) {
            const Interval* best = &intervals[0];
            for (size_t i = 1; i < intervals.size(); ++i) {
                if (intervals[i].peakVal > best->peakVal) {
                    best = &intervals[i];
                }
            }
            // 输出列坐标和亚像素重心行坐标。
            result.push_back(Point2d(static_cast<double>(col), best->centroidRow));
        }
        // 没有有效区间的列不产生激光点。
    }

    return result;
}

std::vector<Point2d> LaserLineExtractor::extractSteger(
    const cv::Mat& image, double sigma, double threshold) {
    std::vector<Point2d> result;

    if (image.channels() != 1) {
        LSC_WARN("LaserLine") << "extractSteger: input image is not grayscale";
        return result;
    }
    if (image.rows < 3 || image.cols < 3) return result;

    // Steger 方法把亮线视为灰度曲面的“脊线”。先转换为 double 并做
    // 高斯平滑，降低像素噪声对一、二阶导数的放大作用。sigma 越大，
    // 抗噪越强，但相邻细线也越容易被融合。
    cv::Mat gray64;
    image.convertTo(gray64, CV_64F);

    cv::Mat smoothed;
    const double safeSigma = std::max(0.1, sigma);
    cv::GaussianBlur(gray64, smoothed, cv::Size(), safeSigma, safeSigma, cv::BORDER_REPLICATE);

    // 梯度 (gx, gy) 描述灰度一阶变化；Hessian
    // [gxx gxy; gxy gyy] 描述局部曲率。亮线横截面中心处沿法向具有
    // 最大负曲率，因此选取绝对值最大的负特征值及其特征向量。
    cv::Mat gx, gy, gxx, gxy, gyy;
    cv::Sobel(smoothed, gx, CV_64F, 1, 0, 3);
    cv::Sobel(smoothed, gy, CV_64F, 0, 1, 3);
    cv::Sobel(smoothed, gxx, CV_64F, 2, 0, 3);
    cv::Sobel(smoothed, gxy, CV_64F, 1, 1, 3);
    cv::Sobel(smoothed, gyy, CV_64F, 0, 2, 3);

    struct Candidate {
        double row = 0.0;
        double score = 0.0;
    };
    std::vector<Candidate> bestPerColumn(static_cast<size_t>(image.cols));

    for (int y = 1; y < image.rows - 1; ++y) {
        for (int x = 1; x < image.cols - 1; ++x) {
            const double intensity = gray64.at<double>(y, x);
            if (intensity <= threshold) continue;

            const double a = gxx.at<double>(y, x);
            const double b = gxy.at<double>(y, x);
            const double c = gyy.at<double>(y, x);

            // 对称 2x2 Hessian 的两个特征值可解析求解，避免为每个
            // 像素创建小矩阵并调用通用特征分解器。
            const double trace = a + c;
            const double diff = a - c;
            const double root = std::sqrt(diff * diff + 4.0 * b * b);
            const double lambda1 = 0.5 * (trace + root);
            const double lambda2 = 0.5 * (trace - root);
            const double lambda = (std::abs(lambda1) > std::abs(lambda2)) ? lambda1 : lambda2;
            if (lambda >= 0.0) continue;

            // 构造 lambda 对应的法向特征向量。第一种写法退化时使用
            // 等价的另一行方程，随后归一化为单位法向 n。
            double nx = b;
            double ny = lambda - a;
            double norm = std::hypot(nx, ny);
            if (norm < 1e-12) {
                nx = lambda - c;
                ny = b;
                norm = std::hypot(nx, ny);
            }
            if (norm < 1e-12) continue;
            nx /= norm;
            ny /= norm;

            // 沿法向对灰度作二阶 Taylor 展开：
            //   I'(t) = grad(I)·n + t * n^T H n = 0
            // 因而 t = -(grad·n)/(n^T H n)。|dx|、|dy|<=0.5 保证极值
            // 位于当前像素单元内，这就是亚像素位置的来源。
            const double denom =
                nx * nx * a + 2.0 * nx * ny * b + ny * ny * c;
            if (std::abs(denom) < 1e-12) continue;

            const double t = -(gx.at<double>(y, x) * nx + gy.at<double>(y, x) * ny) / denom;
            const double dx = t * nx;
            const double dy = t * ny;
            if (std::abs(dx) > 0.5 || std::abs(dy) > 0.5) continue;

            // 当前系统的激光线近似水平，每列只保留一个候选。
            // intensity*|lambda| 同时偏好高亮且横截面曲率清晰的脊线。
            Candidate& best = bestPerColumn[static_cast<size_t>(x)];
            const double score = intensity * std::abs(lambda);
            if (score > best.score) {
                best.row = static_cast<double>(y) + dy;
                best.score = score;
            }
        }
    }

    result.reserve(static_cast<size_t>(image.cols));
    for (int x = 0; x < image.cols; ++x) {
        const Candidate& best = bestPerColumn[static_cast<size_t>(x)];
        if (best.score > 0.0) {
            result.push_back(Point2d(static_cast<double>(x), best.row));
        }
    }

    return result;
}

} // namespace lsc
