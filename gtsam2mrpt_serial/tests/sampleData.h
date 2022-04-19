/* +------------------------------------------------------------------------+
   |                     gtsam2mrpt_serial library                          |
   |                                                                        |
   | Copyright (c) 2022, Jose Luis Blanco Claraco and contributors          |
   | Released under 3-clause BSD license                                    |
   +------------------------------------------------------------------------+ */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstdint>
#include <cstdlib>

gtsam::Values               createTestValues(size_t count = 1);
gtsam::NonlinearFactorGraph createTestGraph(size_t count = 1);
