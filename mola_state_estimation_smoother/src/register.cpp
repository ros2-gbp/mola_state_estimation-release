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
 * @file   register.cpp
 * @brief  Register RTTI classes
 * @author Jose Luis Blanco Claraco
 * @date   Sep 18, 2021
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/initializer.h>

MRPT_INITIALIZER(do_register_navstate_fuse)
{
    MOLA_REGISTER_MODULE(mola::state_estimation_smoother::StateEstimationSmoother);
}
