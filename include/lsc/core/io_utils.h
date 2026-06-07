#pragma once
#include "lsc/core/types.h"
#include <string>

namespace lsc {

// ===== 点云文件读写 =====

/**
 * 保存点云为 PLY 格式（ASCII）
 * @param path  输出文件路径
 * @param cloud 点云数据
 * @param colors 逐点颜色（可选，为空则不写入颜色信息）
 * @return      true=成功
 */
bool savePLY(const std::string& path, const PointCloud& cloud,
             const std::vector<cv::Vec3b>& colors = {});

/**
 * 保存点云为 ASCII XYZ 格式（每行: x y z）
 */
bool saveXYZ(const std::string& path, const PointCloud& cloud);

/**
 * 从 PLY 文件加载点云（仅读取顶点坐标）
 */
bool loadPLY(const std::string& path, PointCloud& cloud);

/**
 * 从 XYZ 文件加载点云
 */
bool loadXYZ(const std::string& path, PointCloud& cloud);

// ===== 标定数据序列化 =====

/**
 * 将相机内参、光平面、移动轴保存为单个 YAML 文件
 * @param yamlPath  输出 YAML 文件路径
 * @param K         相机内参
 * @param lightPlane 光平面参数
 * @param motionAxis 移动轴方向
 * @return          true=成功
 */
bool saveCalibration(const std::string& yamlPath,
                     const CameraIntrinsics& K,
                     const Plane& lightPlane,
                     const Eigen::Vector3d& motionAxis);

} // namespace lsc
