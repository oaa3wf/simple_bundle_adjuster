#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/cost_function.h>
#include <ceres/autodiff_cost_function.h>

namespace SimpleBA{
    struct ReprojectionError 
    {
        ReprojectionError(double observed_x, double observed_y)
            : observed_x(observed_x), observed_y(observed_y) {}

        template <typename T>
        bool operator()(const T* const camFromWorldQ,
                        const T* const camFromWorldT,
                        const T* const pointInWorld,
                        const T* const intrinsics,
                        T* residuals) const 
        {
            // camera[0,1,2] are the angle-axis rotation.
            T pointInCamera[3];
            ceres::QuaternionRotatePoint(camFromWorldQ, pointInWorld, pointInCamera);
            // camera[3,4,5] are the translation.
            pointInCamera[0] += camFromWorldT[0]; 
            pointInCamera[1] += camFromWorldT[1];
            pointInCamera[2] += camFromWorldT[2];

            // Compute the center of distortion. The sign change comes from
            // the camera model that Noah Snavely's Bundler assumes, whereby
            // the camera coordinate system has a negative z axis.
            T xp = pointInCamera[0] / pointInCamera[2];
            T yp = pointInCamera[1] / pointInCamera[2];

            // // Apply second and fourth order radial distortion.
            // const T& l1 = camera[7];
            // const T& l2 = camera[8];
            // T r2 = xp*xp + yp*yp;
            // T distortion = 1.0 + r2  * (l1 + l2  * r2);

            // Compute final projected point position.
            const T& fx = intrinsics[0];
            const T& fy = intrinsics[1];
            const T& cx = intrinsics[2];
            const T& cy = intrinsics[3];

            // TODO: set a maximum value for errors??
            T predicted_x = fx*xp + cx;
            T predicted_y = fy*yp + cy;

            // The error is the difference between the predicted and observed position.
            residuals[0] = predicted_x - T(observed_x);
            residuals[1] = predicted_y - T(observed_y);
            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const double observed_x,
                                            const double observed_y) 
        {
            return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3,  3, 4>(
                        new ReprojectionError(observed_x, observed_y)));
        }

        double observed_x;
        double observed_y;
    };
}