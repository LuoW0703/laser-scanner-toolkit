#pragma once

#include "lsc/core/types.h"

#ifdef LSC_WITH_OPEN3D
#include <memory>
#include <open3d/Open3D.h>
#endif

namespace lsc::viz {

#ifdef LSC_WITH_OPEN3D
inline void visualizePointCloud(const PointCloud& cloud) {
    auto o3dCloud = std::make_shared<open3d::geometry::PointCloud>();
    o3dCloud->points_.reserve(cloud.size());
    for (const auto& p : cloud) {
        o3dCloud->points_.push_back({p.x(), p.y(), p.z()});
    }
    open3d::visualization::DrawGeometries({o3dCloud});
}
#else
inline void visualizePointCloud(const PointCloud&) {}
#endif

} // namespace lsc::viz
