#include "lsc/core/io_utils.h"
#include "lsc/core/laser_line.h"
#include "lsc/core/ray_plane.h"
#include "lsc/calib/motion_axis_calib.h"
#include "lsc/proc/filter.h"
#include "lsc/proc/measurement.h"
#include "lsc/proc/segmentation.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>

#include <opencv2/core.hpp>

namespace {

void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void checkNear(double actual, double expected, double tolerance, const char* message) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << message << ": expected " << expected << ", got " << actual << "\n";
        throw std::runtime_error(message);
    }
}

void testRayPlane() {
    lsc::CameraIntrinsics k;
    k.fx = 100.0;
    k.fy = 100.0;
    k.cx = 10.0;
    k.cy = 20.0;

    const Eigen::Vector3d ray = lsc::pixelToRay({10.0, 20.0}, k);
    checkNear(ray.x(), 0.0, 1e-12, "principal-point ray x");
    checkNear(ray.y(), 0.0, 1e-12, "principal-point ray y");
    checkNear(ray.z(), 1.0, 1e-12, "principal-point ray z");

    lsc::Plane plane;
    plane.C = 1.0;
    plane.D = -50.0;

    lsc::Point3d hit;
    check(lsc::intersectRayPlane({10.0, 20.0}, k, plane, hit), "ray should intersect plane");
    checkNear(hit.x(), 0.0, 1e-12, "intersection x");
    checkNear(hit.y(), 0.0, 1e-12, "intersection y");
    checkNear(hit.z(), 50.0, 1e-12, "intersection z");

    lsc::Plane behind;
    behind.C = 1.0;
    behind.D = 10.0;
    check(!lsc::intersectRayPlane({10.0, 20.0}, k, behind, hit), "behind-camera plane should fail");
}

void testMeasurementAndFilter() {
    const lsc::PointCloud cloud = {
        {0.0, 0.0, 0.0},
        {2.0, 0.0, 1.0},
        {2.0, 3.0, 4.0},
        {-1.0, 3.0, 2.0},
    };

    const auto aabb = lsc::PointCloudMeasurer::computeAABB(cloud);
    checkNear(aabb.minPoint.x(), -1.0, 1e-12, "AABB min x");
    checkNear(aabb.maxPoint.z(), 4.0, 1e-12, "AABB max z");
    checkNear(aabb.dimensions.x(), 3.0, 1e-12, "AABB width");
    checkNear(aabb.dimensions.y(), 3.0, 1e-12, "AABB depth");
    checkNear(aabb.dimensions.z(), 4.0, 1e-12, "AABB height");
    checkNear((aabb.axes - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-12,
              "AABB axes");

    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    lsc::PointCloud rotatedBox;
    for (int sx : {-1, 1}) {
        for (int sy : {-1, 1}) {
            for (int sz : {-1, 1}) {
                rotatedBox.push_back(
                    rotation * Eigen::Vector3d(
                        2.0 * sx, 1.0 * sy, 0.5 * sz));
            }
        }
    }
    const auto obb = lsc::PointCloudMeasurer::computeOBB(rotatedBox);
    checkNear(
        (obb.axes.transpose() * obb.axes -
         Eigen::Matrix3d::Identity()).norm(),
        0.0, 1e-12, "OBB axes orthonormal");
    checkNear(obb.dimensions.prod(), 8.0, 1e-9, "OBB volume");
    for (const auto& point : rotatedBox) {
        check((point.array() >= obb.minPoint.array() - 1e-12).all(),
              "OBB world min contains points");
        check((point.array() <= obb.maxPoint.array() + 1e-12).all(),
              "OBB world max contains points");
    }

    checkNear(lsc::PointCloudMeasurer::computeVolume({}, {0.0, 0.0, 1.0, 0.0}, 1.0),
              0.0, 1e-12, "empty cloud volume");

    const lsc::PointCloud dense = {
        {0.0, 0.0, 0.0},
        {0.1, 0.0, 0.0},
        {1.1, 0.0, 0.0},
        {1.2, 0.0, 0.0},
    };
    const auto downsampled = lsc::PointCloudFilter::voxelDownsample(dense, 1.0);
    check(downsampled.size() == 2, "voxel downsample should keep two occupied voxels");

    const auto invalidPlane =
        lsc::PointCloudSegmenter::ransacPlane(cloud, -0.1, 100, 0.99);
    check(invalidPlane.inlierIndices.empty(),
          "invalid RANSAC threshold must not produce a model");
}

void testMotionAxisInputValidation() {
    lsc::MotionAxisCalibrator calibrator;
    lsc::CameraIntrinsics intrinsics;
    std::vector<cv::Mat> images(2, cv::Mat::zeros(10, 10, CV_8U));
    Eigen::Vector3d axis;
    double rms = 0.0;
    check(!calibrator.calibrate(
              images, intrinsics, cv::Size(3, 3), 1.0, {0.0}, axis, rms),
          "motion-axis positions must match image count");
}

void testPlyIo() {
    const lsc::PointCloud cloud = {
        {1.0, 2.0, 3.0},
        {-4.0, 5.5, 6.0},
    };

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "lsc_test_core_ascii.ply";

    check(lsc::savePLY(path.string(), cloud), "savePLY should succeed");

    lsc::PointCloud loaded;
    check(lsc::loadPLY(path.string(), loaded), "loadPLY should succeed");
    check(loaded.size() == cloud.size(), "loadPLY point count");
    checkNear(loaded[1].x(), -4.0, 1e-12, "loadPLY x");
    checkNear(loaded[1].y(), 5.5, 1e-12, "loadPLY y");
    checkNear(loaded[1].z(), 6.0, 1e-12, "loadPLY z");

    std::filesystem::remove(path);
}

void testLaserLine() {
    cv::Mat image = cv::Mat::zeros(32, 64, CV_8U);
    const double centerRow = 14.25;
    for (int x = 0; x < image.cols; ++x) {
        for (int y = 0; y < image.rows; ++y) {
            const double d = static_cast<double>(y) - centerRow;
            const double value = 220.0 * std::exp(-(d * d) / (2.0 * 1.2 * 1.2));
            image.at<unsigned char>(y, x) = static_cast<unsigned char>(std::round(value));
        }
    }

    lsc::LaserLineExtractor extractor;
    const auto centroid = extractor.extract(image, lsc::LaserMethod::GRAY_CENTROID, 20.0, 1.0);
    check(!centroid.empty(), "gray-centroid extraction should return points");
    checkNear(centroid[centroid.size() / 2].y(), centerRow, 0.25, "gray-centroid row");

    const auto steger = extractor.extract(image, lsc::LaserMethod::STEGER, 20.0, 1.0);
    check(!steger.empty(), "Steger extraction should return points");
    checkNear(steger[steger.size() / 2].y(), centerRow, 0.5, "Steger row");
}

} // namespace

int main() {
    try {
        testRayPlane();
        testMeasurementAndFilter();
        testMotionAxisInputValidation();
        testPlyIo();
        testLaserLine();
    } catch (const std::exception& e) {
        std::cerr << "test_core failed: " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "test_core passed\n";
    return EXIT_SUCCESS;
}
