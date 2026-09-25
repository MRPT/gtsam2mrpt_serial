/* +------------------------------------------------------------------------+
   |                     gtsam2mrpt_serial library                          |
   |                                                                        |
   | Copyright (c) 2022-2026, Jose Luis Blanco Claraco and contributors     |
   | Released under 3-clause BSD license                                    |
   +------------------------------------------------------------------------+ */

#pragma once

#include <gtsam/config.h>

#define GTSAM_VERSION_AT_LEAST(major, minor, patch)                       \
    ((GTSAM_VERSION_MAJOR > (major)) ||                                   \
     (GTSAM_VERSION_MAJOR == (major) && GTSAM_VERSION_MINOR > (minor)) || \
     (GTSAM_VERSION_MAJOR == (major) && GTSAM_VERSION_MINOR == (minor) && \
      GTSAM_VERSION_PATCH >= (patch)))

#define GTSAM_USES_BOOST (!GTSAM_VERSION_AT_LEAST(4, 3, 0))
