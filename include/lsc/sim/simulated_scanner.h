#pragma once

// 仅引入项目内的 types.h，不引入其他项目头文件
#include "lsc/core/types.h"

#include <opencv2/core.hpp>
#include <Eigen/Geometry>
#include <random>
#include <vector>
#include <utility>

namespace lsc {
namespace sim {

// ============================================================
// 阶梯块参数 —— 简单几何体，用于验证扫描精度
// ============================================================
struct StepBlock {
    double width = 50.0;                  // 阶梯块宽度 (mm)，沿 X 方向
    double depth = 50.0;                  // 阶梯块深度 (mm)，沿 Y 方向
    std::vector<double> stepHeights;      // 各级阶梯高度，从基准面起算 (mm)
                                           // 例如 {0, 5, 10, 20} 表示 4 级台阶
};

// ============================================================
// 模拟扫描器
//
// 在无真实硬件时生成仿真扫描数据，包括：
//   - 多姿态棋盘格标定图像
//   - 光平面标定图像（棋盘格 + 激光线）
//   - 移动轴标定图像（平移棋盘格）
//   - 完整扫描序列（图像序列 + ground truth 点云）
//
// 虚拟系统参数：
//   - 相机：1920×1080, 像元 5.5μm, 焦距 16mm, 工作距离 ~300mm
//   - 三角测量角 25°（相机光轴与光平面夹角）
//   - 光平面：0.9063X + 0Y + 0.4226Z - 139.9 = 0
//   - 移动轴沿 Y 方向，行程 100mm
// ============================================================
class SimulatedScanner {
public:
    // ---- 配置参数 ----
    struct Config {
        // 相机参数
        double pixelSize       = 0.0055;    // 像元尺寸 (mm)
        double focalLength     = 16.0;      // 焦距 (mm)
        double workingDistance = 300.0;     // 工作距离 (mm)

        int    imgWidth        = 1920;
        int    imgHeight       = 1080;

        double k1 = -0.12;                  // 径向畸变 k1
        double k2 =  0.03;                  // 径向畸变 k2

        // 系统几何
        double triangulationAngle = 25.0;   // 三角测量角 (度)
        Plane  lightPlane = {0.9063, 0.0, 0.4226, -139.9};  // GT 光平面

        Eigen::Vector3d motionAxis   = {1.0, 0.0, 0.0};  // 移动轴方向
        double          travelRange  = 100.0;             // 行程 (mm)
        double          defaultStep  = 0.5;               // 默认步长 (mm)
        double          scanStartPosition = -50.0;        // 首帧编码器位置 (mm)

        // 棋盘格
        int    chessboardCols = 9;          // 内角点列数
        int    chessboardRows = 6;          // 内角点行数
        double squareSize     = 15.0;       // 方格尺寸 (mm)

        // 噪声模型
        double sensorNoise    = 1.0;        // 传感器噪声标准差 (灰阶)
        double ambientLight   = 5.0;        // 环境光偏移 (灰阶)
        double vibrationNoise = 0.3;        // 振动噪声标准差 (像素/行)

        // 激光线绘制
        int    laserThickness = 5;          // 激光线宽度 (像素)
        int    laserIntensity = 220;        // 激光线峰值亮度 (0-255)
        double laserSigma     = 1.8;        // 高斯模糊 σ，生成高斯截面
    };

    // ---- 构造 ----
    explicit SimulatedScanner(const Config& cfg = Config());

    // ---- 获取配置与内参 ----
    const Config&         getConfig()     const { return cfg_; }
    const CameraIntrinsics& getIntrinsics() const { return intrinsics_; }
    double scanPosition(size_t index) const {
        return cfg_.scanStartPosition +
               static_cast<double>(index) * cfg_.defaultStep;
    }

    // ============================================================
    // 标定图像生成
    // ============================================================

    // 生成多姿态棋盘格图像（用于相机内参标定）
    // numPoses: 姿态数量（默认 20）
    std::vector<cv::Mat> generateCalibImages(int numPoses = 20);

    // 生成光平面标定图像（棋盘格 + 激光线叠加）
    // 每张图同时包含棋盘格和激光线，用于光平面拟合
    std::vector<cv::Mat> generateLightPlaneCalibImages(int numPoses = 10);

    // 生成移动轴标定图像（棋盘格沿 Y 轴平移）
    // 固定棋盘格姿态，仅沿移动轴方向等间距平移
    std::vector<cv::Mat> generateMotionAxisCalibImages(int numSteps = 20);

    // ============================================================
    // 物体扫描
    // ============================================================

    // 对任意三角网格物体执行完整扫描
    // 返回 (图像序列, ground truth 点云)
    std::pair<std::vector<cv::Mat>, PointCloud>
    generateScan(const TriangleMesh& object);

    // 对阶梯块执行完整扫描（便捷重载）
    std::pair<std::vector<cv::Mat>, PointCloud>
    generateScan(const StepBlock& block);

    // 将阶梯块转换为三角网格
    static TriangleMesh stepBlockToMesh(const StepBlock& block,
                                        double workingDistance = 300.0);

private:
    Config            cfg_;
    CameraIntrinsics  intrinsics_;
    std::mt19937      rng_;

    // ---- 内参初始化 ----
    void initIntrinsics();

    // ---- 棋盘格渲染 ----
    // 返回棋盘格内角点的 3D 坐标（棋盘格坐标系，Z=0 平面）
    std::vector<Point3d> getChessboardCorners3d() const;
    // 在指定姿态下渲染棋盘格图像（含畸变）
    cv::Mat renderChessboard(const Eigen::Isometry3d& pose);

    // ---- 激光线渲染 ----
    // 计算光平面与三角网格的交线
    std::vector<Point3d> intersectPlaneMesh(const Plane& plane,
                                            const TriangleMesh& mesh);
    // 将 3D 交线渲染为激光线图像（polylines + GaussianBlur）
    cv::Mat renderLaserOverlay(const std::vector<Point3d>& pts3d,
                               int width, int height);

    // ---- 噪声注入 ----
    void addNoise(cv::Mat& image);

    // ---- 投影 ----
    // 将单个 3D 相机坐标点投影到图像坐标（含畸变）
    Point2d project(const Point3d& p) const;
    // 批量投影
    std::vector<Point2d> projectBatch(const std::vector<Point3d>& pts) const;

    // ---- 随机姿态生成 ----
    // 生成棋盘格可见的随机姿态
    // nominalZ: 名义工作距离, maxAngleDeg: 最大旋转角(度), maxOffset: 最大XY偏移(mm)
    Eigen::Isometry3d randomChessboardPose(double nominalZ,
                                           double maxAngleDeg = 25.0,
                                           double maxOffset   = 50.0);
    // 检查给定姿态下棋盘格是否至少有 minVisibleRatio 的内角点可见
    bool isChessboardVisible(const Eigen::Isometry3d& pose,
                             double minVisibleRatio = 0.6) const;
};

} // namespace sim
} // namespace lsc
