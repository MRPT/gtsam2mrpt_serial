/* +------------------------------------------------------------------------+
   |                     gtsam2mrpt_serial library                          |
   |                                                                        |
   | Copyright (c) 2022, Jose Luis Blanco Claraco and contributors          |
   | Released under 3-clause BSD license                                    |
   +------------------------------------------------------------------------+ */

#pragma once

#include <cstdint>
#include <cstdlib>

// forward decls. for faster compilation in user translation units.
namespace gtsam
{
class Values;
class Value;
class NonlinearFactorGraph;
}  // namespace gtsam
namespace mrpt::serialization
{
class CArchive;
}

namespace gtsam2mrpt_serial
{
/** \name serialize_grp Serialization functions.
 *  Serializes gtsam objects into a binary archive, which can be created with
 * mrpt::serialization::archiveFrom() from a physical I/O stream, socket, etc.
 *
 * @{
 */

mrpt::serialization::CArchive& operator<<(
    mrpt::serialization::CArchive& out, const gtsam::Values& values);

mrpt::serialization::CArchive& operator<<(
    mrpt::serialization::CArchive& out, const gtsam::Value& value);

/** @}
 */

/** \name deserialize_grp De-serialization functions.
 * De-serializes a gtsam::Values from a binary archive, which can be
 * created with mrpt::serialization::archiveFrom() from a physical I/O
 * stream, socket, etc.
 *
 * @{
 */
mrpt::serialization::CArchive& operator>>(
    mrpt::serialization::CArchive& in, gtsam::Values& values);

void deserialize_and_insert(
    mrpt::serialization::CArchive& in, uint64_t key, gtsam::Values& values);

/** @}
 */

}  // namespace gtsam2mrpt_serial
