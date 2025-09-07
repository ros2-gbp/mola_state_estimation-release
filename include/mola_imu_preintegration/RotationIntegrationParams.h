/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   RotationIntegrationParams.h
 * @brief  Parameters for angular velocity integration.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 20, 2021
 */

#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose3D.h>

#include <optional>

namespace mola
{
/** Parameters needed when integrating IMU rotation only.
 *
 *  Refer to:
 *  - Crassidis, J. L. (2006). Sigma-point Kalman filtering for integrated GPS
 * and inertial navigation. IEEE Transactions on Aerospace and Electronic
 * Systems, 42(2), 750-756.
 *  - Forster, C., Carlone, L., Dellaert, F., & Scaramuzza, D. (2015). IMU
 * preintegration on manifold for efficient visual-inertial maximum-a-posteriori
 * estimation. Georgia Institute of Technology.
 *  - Nikolic, J. (2016). Characterisation, calibration, and design of
 * visual-inertial sensor systems for robot navigation (Doctoral dissertation,
 * ETH Zurich).
 *
 * \ingroup mola_imu_preintegration_grp
 */
class RotationIntegrationParams
{
   public:
    RotationIntegrationParams() = default;

    /// Loads all parameters from a YAML map node.
    void load_from(const mrpt::containers::yaml& cfg);

    /// Gyroscope (initial or constant) bias, in the local IMU frame of
    /// reference (units: rad/s).
    mrpt::math::TVector3D gyroBias = {.0, .0, .0};

    /// Gyroscope covariance (units of sigma are rad/s/√Hz )
    mrpt::math::CMatrixDouble33 gyroCov =
        mrpt::math::CMatrixDouble33::Identity();

    /// If provided, defines an IMU placed at a pose different than the
    /// vehicle origin of coordinates (Default: IMU used as reference of the
    /// vehicle frame, i.e. sensorPose = SE(3) identity I_{4x4}).
    std::optional<mrpt::poses::CPose3D> sensorPose;
};

}  // namespace mola
