#pragma once

#include <array>
#include <string>
#include <unordered_map>

#include <ceres/problem.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "cost_functors.h"

namespace SimpleBA
{

    struct Camera
    {
        Camera(const std::array<double, 4>& camFromWorldQ_in, 
        const std::array<double, 3>& camFromWorldT_in,
        const std::string& id_in, const std::string& intrinsicsId_in,
        bool isFixed_in)
        {
            id = id_in;
            intrinsicsId = intrinsicsId_in;
            camFromWorldQ = camFromWorldQ_in;
            camFromWorldT = camFromWorldT_in;
            isFixed = isFixed_in;
        }

        Camera()
        {}
        std::string id;
        std::string intrinsicsId;
        // store as w, x, y, z
        std::array<double, 4> camFromWorldQ;
        std::array<double, 3> camFromWorldT;
        bool isFixed;
    };

    struct Intrinsics
    {
        Intrinsics(const double& fx_in, 
        const double& fy_in,
        const double& cx_in,
        const double& cy_in,
        const std::string& id_in,
        bool isFixed_in)
        {
            cameraParameters[0] = fx_in;
            cameraParameters[1] = fy_in;
            cameraParameters[2] = cx_in;
            cameraParameters[3] = cy_in;
            id = id_in;
            isFixed = isFixed_in;
        }

        Intrinsics()
        {}
        bool isFixed;
        std::array<double, 4> cameraParameters;
        std::string id;
    };

    struct Observation
    {
        Observation(const double& x_in,
        const double& y_in,
        const std::string& id_in,
        const std::string& cameraId_in,
        const std::string& pointId_in)
        {
            id = id_in;
            cameraId = cameraId_in;
            pointId = pointId_in;
            x = x_in;
            y = y_in;
        }

        Observation()
        {}

        std::string id;
        std::string cameraId;
        std::string pointId;
        double x;
        double y;
    };

    struct Point
    {
        Point(const double& x_in,
        const double& y_in,
        const double& z_in,
        const std::string& id_in,
        bool isFixed_in)
        {
            id = id_in;
            point3d[0] = x_in;
            point3d[1] = y_in;
            point3d[2] = z_in;
            isFixed = isFixed_in;
        }
        Point()
        {}
        std::string id;
        std::array<double, 3> point3d;
        bool isFixed;
    };

    struct SyntheticSceneGenerator
    {
        SyntheticSceneGenerator() {};
        std::unordered_map<std::string, SimpleBA::Intrinsics> intrinsics_map;
        std::unordered_map<std::string, SimpleBA::Camera> cameras_map;
        std::unordered_map<std::string, SimpleBA::Point> points_map;
        std::unordered_map<std::string, SimpleBA::Observation> observations_map;
    };

    class BundleAdjuster
    {
    public:
        BundleAdjuster()
        {
            problem = std::make_shared<ceres::Problem>();
        };

        std::unordered_map<std::string, SimpleBA::Intrinsics> intrinsics_map;
        std::unordered_map<std::string, SimpleBA::Camera> cameras_map;
        std::unordered_map<std::string, SimpleBA::Point> points_map;
        std::unordered_map<std::string, SimpleBA::Observation> observations_map;
        std::shared_ptr<ceres::Problem> problem; 

        void addCameraIntrinsics(const SimpleBA::Intrinsics& intrinsics)
        {
            intrinsics_map[intrinsics.id] = intrinsics;
        }
        void addCamera(const SimpleBA::Camera& camera)
        {
            cameras_map[camera.id] = camera;
        }
        void addPoint(const SimpleBA::Point& point)
        {
            points_map[point.id] = point;
        }
        void addObservation(const SimpleBA::Observation& observation)
        {
            observations_map[observation.id] = observation;
        }

        //void writePoints(const std::unordered_map<std::string, SimpleBA::Point>& points_map_in);
        //void writeCameras(const std::unordered_map<std::string, SimpleBA::Camera>& cameras_map_in);
        //void writeIntrinsics(const std::unordered_map<std::string, SimpleBA::Intrinsics>& intrinsics_map_in);

        bool run();
    };
}