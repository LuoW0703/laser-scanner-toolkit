#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "lsc/core/ray_plane.h"
#include "lsc/core/types.h"
#include "lsc/proc/filter.h"
#include "lsc/proc/measurement.h"

namespace py = pybind11;

PYBIND11_MODULE(lsc, m) {
    m.doc() = "Laser Scanner Toolkit Python bindings";

    py::class_<lsc::CameraIntrinsics>(m, "CameraIntrinsics")
        .def(py::init<>())
        .def_readwrite("fx", &lsc::CameraIntrinsics::fx)
        .def_readwrite("fy", &lsc::CameraIntrinsics::fy)
        .def_readwrite("cx", &lsc::CameraIntrinsics::cx)
        .def_readwrite("cy", &lsc::CameraIntrinsics::cy)
        .def_readwrite("k1", &lsc::CameraIntrinsics::k1)
        .def_readwrite("k2", &lsc::CameraIntrinsics::k2)
        .def_readwrite("k3", &lsc::CameraIntrinsics::k3)
        .def_readwrite("p1", &lsc::CameraIntrinsics::p1)
        .def_readwrite("p2", &lsc::CameraIntrinsics::p2)
        .def_readwrite("img_width", &lsc::CameraIntrinsics::imgWidth)
        .def_readwrite("img_height", &lsc::CameraIntrinsics::imgHeight);

    py::class_<lsc::Plane>(m, "Plane")
        .def(py::init<>())
        .def_readwrite("a", &lsc::Plane::A)
        .def_readwrite("b", &lsc::Plane::B)
        .def_readwrite("c", &lsc::Plane::C)
        .def_readwrite("d", &lsc::Plane::D)
        .def("normal", &lsc::Plane::normal)
        .def("normalize", &lsc::Plane::normalize)
        .def("distance_to_point", &lsc::Plane::distanceToPoint);

    py::class_<lsc::PointCloudMeasurer::BoundingBox>(m, "BoundingBox")
        .def_readonly("min_point", &lsc::PointCloudMeasurer::BoundingBox::minPoint)
        .def_readonly("max_point", &lsc::PointCloudMeasurer::BoundingBox::maxPoint)
        .def_readonly("center", &lsc::PointCloudMeasurer::BoundingBox::center)
        .def_readonly("dimensions", &lsc::PointCloudMeasurer::BoundingBox::dimensions)
        .def_readonly("axes", &lsc::PointCloudMeasurer::BoundingBox::axes);

    m.def("pixel_to_ray", &lsc::pixelToRay);
    m.def("voxel_downsample", &lsc::PointCloudFilter::voxelDownsample);
    m.def("compute_aabb", &lsc::PointCloudMeasurer::computeAABB);
}
