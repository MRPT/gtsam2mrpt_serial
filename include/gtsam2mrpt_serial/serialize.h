/* +------------------------------------------------------------------------+
   |                     gtsam2mrpt_serial library                          |
   |                                                                        |
   | Copyright (c) 2022-2026, Jose Luis Blanco Claraco and contributors     |
   | Released under 3-clause BSD license                                    |
   +------------------------------------------------------------------------+ */

#pragma once

#include <cstdint>
#include <cstdlib>

// Forward declarations for faster compilation in user translation units.
namespace gtsam
{
class Values;
class Value;
class NonlinearFactorGraph;
class NonlinearFactor;
}  // namespace gtsam
namespace mrpt::serialization
{
class CArchive;
}

namespace gtsam2mrpt_serial
{
/** \name Serialization
 *  Binary serialization of GTSAM objects to an mrpt::serialization::CArchive.
 *
 *  Archives can be created with mrpt::serialization::archiveFrom() wrapping
 *  any mrpt::io stream (files, sockets, pipes, etc.), with optional transparent
 *  gzip / zstd compression via mrpt::io::CCompressedOutputStream.
 *
 *  The binary format is portable across architectures, endianness, and OS, and
 *  supports versioning so future library versions can read older files.
 *
 * @{
 */

mrpt::serialization::CArchive& operator<<(
    mrpt::serialization::CArchive& out, const gtsam::Values& values);

mrpt::serialization::CArchive& operator<<(
    mrpt::serialization::CArchive& out, const gtsam::Value& value);

mrpt::serialization::CArchive& operator<<(
    mrpt::serialization::CArchive& out, const gtsam::NonlinearFactorGraph& fg);

mrpt::serialization::CArchive& operator<<(
    mrpt::serialization::CArchive& out, const gtsam::NonlinearFactor& f);

/** @} */

/** \name De-serialization
 *  Binary de-serialization of GTSAM objects from an
 * mrpt::serialization::CArchive.
 *
 * @{
 */

mrpt::serialization::CArchive& operator>>(
    mrpt::serialization::CArchive& in, gtsam::Values& values);

/** Deserializes a single gtsam::Value from \a in and inserts it into \a values
 *  under the given \a key.  The type tag is read from the stream. */
void deserialize_and_insert(
    mrpt::serialization::CArchive& in, uint64_t key, gtsam::Values& values);

mrpt::serialization::CArchive& operator>>(
    mrpt::serialization::CArchive& in, gtsam::NonlinearFactorGraph& fg);

/** Deserializes one factor from \a in and returns it as a raw, heap-allocated
 *  pointer.  The caller takes ownership and is responsible for deletion
 *  (typically by wrapping the result immediately in a shared_ptr or
 *  boost::shared_ptr). */
[[nodiscard]] gtsam::NonlinearFactor* deserialize_factor(
    mrpt::serialization::CArchive& in);

/** @} */

}  // namespace gtsam2mrpt_serial