#include "lsc/core/io_utils.h"
#include "lsc/core/log.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace lsc {

namespace {

constexpr size_t kMaxPointCount = 50'000'000;
constexpr uintmax_t kMaxFileSizeBytes = 500'000'000;
constexpr size_t kMaxPlyProperties = 128;
constexpr size_t kMaxLineLength = 1'000'000;

bool isRegularFileWithinLimit(const std::string& path) {
    std::error_code ec;
    const std::filesystem::path fsPath(path);
    if (!std::filesystem::is_regular_file(fsPath, ec)) {
        LSC_ERROR("IO") << "not a regular file: " << path;
        return false;
    }

    const uintmax_t size = std::filesystem::file_size(fsPath, ec);
    if (ec) {
        LSC_ERROR("IO") << "cannot read file size: " << path;
        return false;
    }
    if (size > kMaxFileSizeBytes) {
        LSC_ERROR("IO") << "file too large: " << size << " bytes";
        return false;
    }
    return true;
}

bool isFinitePoint(double x, double y, double z) {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

} // namespace

bool savePLY(const std::string& path, const PointCloud& cloud,
             const std::vector<cv::Vec3b>& colors) {
    if (!colors.empty() && colors.size() != cloud.size()) {
        LSC_ERROR("IO") << "savePLY: colors size does not match point cloud size";
        return false;
    }

    std::ofstream f(path);
    if (!f.is_open()) return false;

    f << "ply\n";
    f << "format ascii 1.0\n";
    f << "element vertex " << cloud.size() << "\n";
    f << "property float x\n";
    f << "property float y\n";
    f << "property float z\n";
    if (!colors.empty()) {
        f << "property uchar red\n";
        f << "property uchar green\n";
        f << "property uchar blue\n";
    }
    f << "end_header\n";
    for (size_t i = 0; i < cloud.size(); ++i) {
        f << cloud[i].x() << " " << cloud[i].y() << " " << cloud[i].z();
        if (!colors.empty()) {
            f << " " << (int)colors[i][2] << " " << (int)colors[i][1] << " " << (int)colors[i][0];
        }
        f << "\n";
    }
    return true;
}

bool saveXYZ(const std::string& path, const PointCloud& cloud) {
    std::ofstream f(path);
    if (!f.is_open()) return false;
    for (const auto& p : cloud) {
        f << p.x() << " " << p.y() << " " << p.z() << "\n";
    }
    return true;
}

bool loadPLY(const std::string& path, PointCloud& cloud) {
    if (!isRegularFileWithinLimit(path)) return false;

    std::ifstream f(path);
    if (!f.is_open()) return false;

    std::string line;
    if (!std::getline(f, line) || line != "ply") return false;

    bool ascii = false;
    bool inVertexElement = false;
    bool headerEnded = false;
    size_t vertexCount = 0;
    std::vector<std::string> vertexProperties;

    while (std::getline(f, line)) {
        if (line.size() > kMaxLineLength) {
            LSC_ERROR("IO") << "loadPLY: header line exceeds safety limit";
            return false;
        }

        if (line == "end_header") {
            headerEnded = true;
            break;
        }

        std::istringstream iss(line);
        std::string keyword;
        iss >> keyword;

        if (keyword == "format") {
            std::string format;
            iss >> format;
            ascii = (format == "ascii");
        } else if (keyword == "element") {
            std::string elementName;
            size_t count = 0;
            iss >> elementName >> count;
            inVertexElement = (elementName == "vertex");
            if (inVertexElement) {
                if (count > kMaxPointCount) {
                    LSC_ERROR("IO") << "loadPLY: vertex count exceeds safety limit: " << count;
                    return false;
                }
                vertexCount = count;
                vertexProperties.clear();
            }
        } else if (keyword == "property" && inVertexElement) {
            std::string type;
            std::string name;
            iss >> type >> name;
            if (!name.empty()) vertexProperties.push_back(name);
            if (vertexProperties.size() > kMaxPlyProperties) {
                LSC_ERROR("IO") << "loadPLY: too many vertex properties";
                return false;
            }
        }
    }

    if (!headerEnded || !ascii || vertexCount == 0 || vertexProperties.empty()) return false;

    auto propertyIndex = [&](const std::string& name) -> int {
        for (size_t i = 0; i < vertexProperties.size(); ++i) {
            if (vertexProperties[i] == name) return static_cast<int>(i);
        }
        return -1;
    };

    const int xIndex = propertyIndex("x");
    const int yIndex = propertyIndex("y");
    const int zIndex = propertyIndex("z");
    if (xIndex < 0 || yIndex < 0 || zIndex < 0) return false;

    PointCloud loaded;
    loaded.reserve(vertexCount);

    for (size_t i = 0; i < vertexCount; ++i) {
        if (!std::getline(f, line)) return false;
        if (line.size() > kMaxLineLength) {
            LSC_ERROR("IO") << "loadPLY: vertex line exceeds safety limit";
            return false;
        }

        std::istringstream iss(line);
        std::vector<double> values(vertexProperties.size(), 0.0);
        for (double& value : values) {
            if (!(iss >> value)) return false;
        }

        const double x = values[static_cast<size_t>(xIndex)];
        const double y = values[static_cast<size_t>(yIndex)];
        const double z = values[static_cast<size_t>(zIndex)];
        if (!isFinitePoint(x, y, z)) {
            LSC_ERROR("IO") << "loadPLY: non-finite vertex coordinate";
            return false;
        }

        loaded.push_back({x, y, z});
    }

    cloud = std::move(loaded);
    return true;
}

bool loadXYZ(const std::string& path, PointCloud& cloud) {
    if (!isRegularFileWithinLimit(path)) return false;

    std::ifstream f(path);
    if (!f.is_open()) return false;

    PointCloud loaded;
    loaded.reserve(1024);

    double x, y, z;
    while (f >> x >> y >> z) {
        if (loaded.size() >= kMaxPointCount) {
            LSC_ERROR("IO") << "loadXYZ: point count exceeds safety limit";
            return false;
        }
        if (!isFinitePoint(x, y, z)) {
            LSC_ERROR("IO") << "loadXYZ: non-finite point coordinate";
            return false;
        }
        loaded.push_back({x, y, z});
    }

    cloud = std::move(loaded);
    return true;
}

bool saveCalibration(const std::string& yamlPath,
                     const CameraIntrinsics& K,
                     const Plane& lightPlane,
                     const Eigen::Vector3d& motionAxis) {
    cv::FileStorage fs(yamlPath, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;
    fs << "fx" << K.fx << "fy" << K.fy << "cx" << K.cx << "cy" << K.cy;
    fs << "k1" << K.k1 << "k2" << K.k2 << "k3" << K.k3;
    fs << "p1" << K.p1 << "p2" << K.p2;
    fs << "imgWidth" << K.imgWidth << "imgHeight" << K.imgHeight;
    fs << "planeA" << lightPlane.A << "planeB" << lightPlane.B;
    fs << "planeC" << lightPlane.C << "planeD" << lightPlane.D;
    fs << "axisX" << motionAxis.x() << "axisY" << motionAxis.y() << "axisZ" << motionAxis.z();
    fs.release();
    return true;
}

} // namespace lsc
