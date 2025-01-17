#pragma once

#include <functional>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/cost_function.h>
#include <ceres/autodiff_cost_function.h>
#include <Eigen/Geometry>

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

    struct ImuReading
    {
        ImuReading()
        {}
        unsigned long int time;
        std::array<double, 3> accel;
        std::array<double, 3> gyro;
        //std::array<double, 3> accelBias;
        //std::array<double, 3> gyroBias;
        //std::array<double, 3> gravity;
        std::array<double, 4> imuFromCamQ;
        std::array<double, 4> imuFromCamT;
    };

    struct PreIntegratedImuError
    {
        PreIntegratedImuError(const std::vector<ImuReading>& imuReadings, const unsigned long int& t0, const unsigned long int& t1)
        {
            m_imuReadings = imuReadings;
            m_t0 = t0;
            m_t1 = t1;
            // sort imu readings in time
            std::sort(m_imuReadings.begin(), m_imuReadings.end(),
                [](const ImuReading& a, const ImuReading& b)
                {
                    return a.time < b.time;
                }
            );
        };

        template <typename T>
        bool operator()(const T* const camFromWorldQ_i,
            const T* const camFromWorldT_i,
            const T* const camFromWorldQ_j,
            const T* const camFromWorldT_j,
            const T* const velocity_i,
            const T* const velocity_j,
            const T* const accelBias,
            const T* const gyroBias,
            const T* const gravity,
            T* residuals) const
        {
            // TODO: Might need to transform readings from imu frame to cam frame or vice versa
            const Eigen::Quaterniond eigenImuFromCamQ(m_imuReadings[0].imuFromCamQ[3], m_imuReadings[0].imuFromCamQ[0],
                m_imuReadings[0].imuFromCamQ[1], m_imuReadings[0].imuFromCamQ[2]);
            const Eigen::Vector3d eigenImuFromCamT(m_imuReadings[0].imuFromCamT[0], m_imuReadings[0].imuFromCamT[1], m_imuReadings[0].imuFromCamT[2]);

            // TODO: Need to handle case where camera sampling time and imu sampling time do not align

            // TODO: validate the order (w,x,y,z) between quaternions for eigen and ceres
            Eigen::Map<const Eigen::Quaternion<T>>eigenCamFromWorldQ_i(camFromWorldQ_i);
            const Eigen::AngleAxis<T> eigenCamFromWorldAngleAxis_i(camFromWorldQ_i);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>>eigenCamFromWorldT_i(camFromWorldT_i);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>>eigenVelocity_i(velocity_i);

            const Eigen::Quaternion<T> eigenImuFromWorldQ_i = eigenImuFromCamQ.cast<T>() * eigenCamFromWorldQ_i;
            const Eigen::AngleAxis<T> eigenImuFromWorldAngleAxis_i(eigenImuFromWorldQ_i);
            const Eigen::Matrix<T, 3, 1> eigenImuFromWorldT_i = eigenImuFromCamQ.cast<T>() * eigenCamFromWorldT_i * eigenImuFromCamQ.cast<T>().inverse() + eigenImuFromCamT.cast<T>();
            const Eigen::Matrix<T, 3, 1> eigenImuVelocity_i = eigenImuFromCamQ.cast<T> *eigenImuVelocity_i * eigenImuFromCamQ.cast<T>().inverse();

            Eigen::Map<const Eigen::Quaternion<T>>eigenCamFromWorldQ_j(camFromWorldQ_j);
            const Eigen::AngleAxis<T> eigenCamFromWorldAngleAxis_j(camFromWorldQ_j);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>>eigenCamFromWorldT_j(camFromWorldT_j);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>>eigenVelocity_j(velocity_j);

            const Eigen::Quaternion<T> eigenImuFromWorldQ_j = eigenImuFromCamQ.cast<T>() * eigenCamFromWorldQ_j;
            const Eigen::AngleAxis<T> eigenImuFromWorldAngleAxis_j(eigenImuFromWorldQ_j);
            const Eigen::Matrix<T, 3, 1> eigenImuFromWorldT_j = eigenImuFromCamQ.cast<T>() * eigenCamFromWorldT_j * eigenImuFromCamQ.cast<T>().inverse() + eigenImuFromCamT.cast<T>();
            const Eigen::Matrix<T, 3, 1> eigenImuVelocity_j = eigenImuFromCamQ.cast<T> *eigenImuVelocity_j * eigenImuFromCamQ.cast<T>().inverse();

            Eigen::Map<const Eigen::Matrix<T, 3, 1>>eigenGravity(gravity);

            Eigen::Vector3d runningOmega = Eigen::Vector3d::Zero();
            Eigen::Vector3d runningVelocity = Eigen::Vector3d::Zero();
            Eigen::Vector3d runningPosition = Eigen::Vector3d::Zero();

            // TODO: this is wrong, also needs to be converted to seconds properly
            const double t_ij = static_cast<double>(m_imuReadings[m_imuReadings.size() - 1].time - m_imuReadings[0].time);
            for (size_t i = 1; i < m_imuReadings.size(); i++)
            {
                const auto& imuReading_i = m_imuReadings[i];
                const auto& imuReading_i_minus_one = m_imuReadings[i - 1];
                const double delta_t = imuReading_i.time - imuReading_i_minus_one.time;

                Eigen::Map<const Eigen::Matrix<T, 3, 1>>eigenAccelBias(accelBias);
                Eigen::Map<const Eigen::Matrix<T, 3, 1>>eigenGyroBias(gyroBias);
                Eigen::Map<const Eigen::Matrix<T, 3, 1>>eigenGravity(gravity);

                const Eigen::Vector3d omega(imuReading_i.gyro[0], imuReading_i.gyro[1], imuReading_i.gyro[2]);
                runningOmega += (omega - eigenGyroBias.cast<double>()) * delta_t;

                const Eigen::Vector3d accel(imuReading_i.accel[0], imuReading_i.accel[1], imuReading_i.accel[2]);
                const Eigen::Vector3d preRotationVelocity = (accel - eigenAccelBias.cast<double>()) * delta_t;
                const double angle = runningOmega.norm();
                const Eigen::Vector3d axis = runningOmega.normalized();
                const Eigen::AngleAxisd runningOmegaAngleAxis(angle, axis);
                runningVelocity += runningOmegaAngleAxis.toRotationMatrix() * preRotationVelocity;

                runningPosition += 3.0 / 2.0 * runningOmegaAngleAxis.toRotationMatrix() * preRotationVelocity * delta_t;
            }

            const double runningAngle = runningOmega.norm();
            const Eigen::Vector3d runningAxis = runningOmega.normalized();
            const Eigen::AngleAxisd runningOmegaAxisAngle(runningAngle, runningAxis);

            const Eigen::Matrix<T, 3, 3> Ri_T_Rj = eigenImuFromWorldAngleAxis_i.toRotationMatrix().transpose() * eigenImuFromWorldAngleAxis_j.toRotationMatrix();
            const Eigen::AngleAxis<T> errorAxisAngle(runningOmegaAxisAngle.toRotationMatrix().cast<T>().transpose() * Ri_T_Rj);
            const Eigen::Matrix<T, 3, 1> logError = errorAxisAngle.axis() * errorAxisAngle.angle();

            const Eigen::Matrix<T, 3, 1> actual_delta_v_ij = eigenImuFromWorldAngleAxis_i.toRotationMatrix().transpose() * (eigenImuVelocity_j - eigenImuVelocity_i - eigenGravity * static_cast<T>(t_ij));
            const Eigen::Matrix<T, 3, 1> delta_v_ij_error = actual_delta_v_ij - runningVelocity;

            const Eigen::Matrix<T, 3, 1> actualPositionDifference = eigenImuFromWorldT_j - eigenImuFromWorldT_i;
            const Eigen::Matrix<T, 3, 1> velocityCorrectedPositionDifference = actualPositionDifference - eigenVelocity_i * static_cast<T>(t_ij);
            const Eigen::Matrix<T, 3, 1> preRotatedPositionDifference = velocityCorrectedPositionDifference - eigenGravity * static_cast<T>(0.5*t_ij * t_ij);
            const Eigen::Matrix<T, 3, 1> rotatedPositionDifference = eigenImuFromWorldAngleAxis_i.toRotationMatrix().transpose() * preRotatedPositionDifference;
            const Eigen::Matrix<T, 3, 1> delta_p_ij_error = rotatedPositionDifference - runningPosition;

            residuals[0] = logError(0, 0);
            residuals[1] = logError(1, 0);
            residuals[2] = logError(2, 0);

            residuals[3] = delta_v_ij_error(0, 0);
            residuals[4] = delta_v_ij_error(1, 0);
            residuals[5] = delta_v_ij_error(2, 0);

            residuals[6] = delta_p_ij_error(0, 0);
            residuals[7] = delta_p_ij_error(1, 0);
            residuals[8] = delta_p_ij_error(2, 0);


            return true
        }

        std::vector<ImuReading> m_imuReadings;
        unsigned long int m_t0;
        unsigned long int m_t1;
    };
}