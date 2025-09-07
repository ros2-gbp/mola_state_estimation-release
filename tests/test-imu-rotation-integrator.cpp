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
 * @file   test-imu-rotation-integrator.cpp
 * @brief  Unit tests for mola-imu-preintegration functionality
 * @author Jose Luis Blanco Claraco
 * @date   Sep 21, 2021
 */

#include <mola_imu_preintegration/RotationIntegrator.h>
#include <mrpt/poses/Lie/SE.h>

#include <iostream>

using namespace std::string_literals;

static const char* yamlRotIntParams1 =
    R"###(# Config for gtsam::RotationIntegrationParams
gyroBias: [-1.0e-4, 2.0e-4, -3.0e-4]
sensorLocationInVehicle:
  quaternion: [0.0, 0.0, 0.0, 1.0]
  translation: [0.0, 0.0, 0.0]
)###";

static void test_rotation_integration()
{
    mola::RotationIntegrator ri;
    ri.initialize(mrpt::containers::yaml::FromText(yamlRotIntParams1));

    ASSERT_EQUAL_(ri.params_.gyroBias.x, -1.0e-4);
    ASSERT_EQUAL_(ri.params_.gyroBias.y, +2.0e-4);
    ASSERT_EQUAL_(ri.params_.gyroBias.z, -3.0e-4);

    ASSERT_(!ri.params_.sensorPose.has_value());  // since it's the Identity.
    // const auto gtPose = mrpt::poses::CPose3D::Identity();
    // ASSERT_LT_(mrpt::poses::Lie::SE<3>::log(*ri.params_.sensorPose -
    // gtPose).norm(),1e-6);

    // Initial state:
    auto lambdaAssertInitialState = [&ri]()
    {
        const auto& s = ri.current_integration_state();
        ASSERT_LT_((s.deltaRij_ - mrpt::math::CMatrixDouble33::Identity()).norm(), 1e-6);
        ASSERT_EQUAL_(s.deltaTij_, .0);
    };

    lambdaAssertInitialState();

    // Integrate some readings:
    for (int t = 0; t < 2000; t++)
    {
        const double dt = 1e-3;
        ri.integrate_measurement({0, 0, 3.0}, dt);
    }
    // Final state:
    {
        const auto& s     = ri.current_integration_state();
        const auto  gtRot = mrpt::poses::CPose3D::FromYawPitchRoll(6.0, 0, 0);

        ASSERT_LT_((s.deltaRij_ - gtRot.getRotationMatrix()).norm(), 1e-2);
        ASSERT_NEAR_(s.deltaTij_, 2.0, 1e-3);
    }

    // reset:
    ri.reset_integration();
    lambdaAssertInitialState();
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_rotation_integration();

        std::cout << "Test successful." << std::endl;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
