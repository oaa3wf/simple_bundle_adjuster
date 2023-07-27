#include <pybind11/pybind11.h>
#include <simpleBA.h>

namespace py = pybind11;

PYBIND11_MODULE(simpleBA, m)
{
    py::class_<SimpleBA::Camera>(m, "Camera")
        .def(py::init<const std::array<double, 4>&, 
        const std::array<double, 3>&,
        const std::string&,
        const std::string&,
        bool>())
        .def_readwrite("id", &SimpleBA::Camera::id)
        .def_readwrite("intrinsicsId", &SimpleBA::Camera::intrinsicsId)
        .def_readwrite("camFromWorldQ", &SimpleBA::Camera::camFromWorldQ)
        .def_readwrite("camFromWorldT", &SimpleBA::Camera::camFromWorldT)
        .def_readwrite("isFixed", &SimpleBA::Camera::isFixed);

    py::class_<SimpleBA::Intrinsics>(m, "Intrinsics")
        .def(py::init<const double&, 
        const double&,
        const double&,
        const double&,
        const std::string&,
        bool>())
        .def_readwrite("id", &SimpleBA::Intrinsics::id)
        .def_readwrite("intrinsicsId", &SimpleBA::Intrinsics::camParameters)
        .def_readwrite("isFixed", &SimpleBA::Intrinsics::isFixed);

    py::class_<SimpleBA::Observation>(m, "Observation")
        .def(py::init<const double&,
        const double&,
        const std::string&,
        const std::string&,
        const std::string&>())
        .def_readwrite("id", &SimpleBA::Observation::id)
        .def_readwrite("cameraId", &SimpleBA::Observation::cameraId)
        .def_readwrite("pointId", &SimpleBA::Observation::pointId)
        .def_readwrite("x", &SimpleBA::Observation::x)
        .def_readwrite("y", &SimpleBA::Observation::y);

    py::class_<SimpleBA::Point>(m, "Point")
        .def(py::init<const double&,
        const double&,
        const double&,
        const std::string&,
        bool>())
        .def_readwrite("id", &SimpleBA::Point::id)
        .def_readwrite("point3d", &SimpleBA::Point::point3d)
        .def_readwrite("isFixed", &SimpleBA::Point::isFixed);

    py::class_<SimpleBA::BundleAdjuster>(m, "BundleAdjuster")
        .def(py::init())
        .def_readwrite("intrinsics_map", &SimpleBA::BundleAdjuster::intrinsics_map)
        .def_readwrite("cameras_maps", &SimpleBA::BundleAdjuster::cameras_maps)
        .def_readwrite("points_map", &SimpleBA::BundleAdjuster::points_map)
        .def_readwrite("observations_map", &SimpleBA::BundleAdjuster::observations_map)
        .def("run", &SimpleBA::BundleAdjuster::run)
        .def("addCameraIntrinsics", &SimpleBA::BundleAdjuster::addCameraIntrinsics)
        .def("addCamera", &SimpleBA::BundleAdjuster::addCamera)
        .def("addPoint", &SimpleBA::BundleAdjuster::addPoint)
        .def("addObservation", &SimpleBA::BundleAdjuster::addObservation);
}