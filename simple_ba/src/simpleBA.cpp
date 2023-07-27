#include "simpleBA.h"

#include <vector>

#include <ceres/manifold.h>

namespace SimpleBA
{
    bool BundleAdjuster::run()
    {
        // create problem
        // make copy of variables
        auto tmp_observations_map = this->observations_map;
        auto tmp_points_map = this->points_map;
        auto tmp_cameras_map = this->cameras_map;
        auto tmp_intrinsics_map = this->intrinsics_map;

        for (auto& id_observation_pair : tmp_observations_map)
        {
            auto& observation = id_observation_pair.second;
            if(tmp_cameras_map.count(observation.cameraId) == 0)
            {
                return false;
            }

            if(tmp_points_map.count(observation.pointId) == 0)
            {
                return false;
            }

            auto& camera = tmp_cameras_map[observation.cameraId];
            auto& point = tmp_points_map[observation.pointId];
            if(tmp_intrinsics_map.count(camera.intrinsicsId) == 0)
            {
                return false;
            }
            auto& intrinsics = tmp_intrinsics_map[camera.intrinsicsId];

            ceres::CostFunction* newCostFunction = ReprojectionError::Create(observation.x, observation.y);
            problem->AddResidualBlock(newCostFunction,
            nullptr,
            camera.camFromWorldQ.data(),
            camera.camFromWorldT.data(),
            point.point3d.data(),
            intrinsics.cameraParameters.data());

            problem->SetManifold(camera.camFromWorldQ.data(), new ceres::QuaternionManifold());

            if (camera.isFixed)
            {
                problem->SetParameterBlockConstant(camera.camFromWorldQ.data());
                problem->SetParameterBlockConstant(camera.camFromWorldT.data());
            }

            if (point.isFixed)
            {
                problem->SetParameterBlockConstant(point.point3d.data());
            }

            if (intrinsics.isFixed)
            {
                problem->SetParameterBlockConstant(intrinsics.cameraParameters.data());
            }
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        //options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
        options.max_num_iterations = 25;
        options.gradient_tolerance = 1e-16;
        options.function_tolerance = 1e-16;
        options.parameter_tolerance = 1e-16;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &(*problem), &summary);

        if (summary.initial_cost > summary.final_cost)
        {
            this->cameras_map = tmp_cameras_map;
            this->points_map = tmp_points_map;
            this->intrinsics_map = tmp_intrinsics_map;
        }

        return true;
    }
}