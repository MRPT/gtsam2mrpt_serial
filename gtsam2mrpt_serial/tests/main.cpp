/* +------------------------------------------------------------------------+
   |                     gtsam2mrpt_serial library                          |
   |                                                                        |
   | Copyright (c) 2022, Jose Luis Blanco Claraco and contributors          |
   | Released under 3-clause BSD license                                    |
   +------------------------------------------------------------------------+ */

#include <gtsam/base/serialization.h>
#include <gtsam2mrpt_serial/serialize.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTimeLogger.h>

#include <functional>
#include <iostream>

#include "boost-exports.h"
#include "sampleData.h"

// --------------
static void testSerializeValues(size_t n)
{
    using namespace gtsam2mrpt_serial;  // expose the << & >> operators;

    const gtsam::Values v = createTestValues(n);

    // save values to binary stream:
    // (Replace this with CFileGZOutputStream to save to a real file)
    mrpt::io::CMemoryStream buf;
    {
        auto arch = mrpt::serialization::archiveFrom(buf);
        arch << v;
    }
    buf.Seek(0);

    // Debug:
    // buf.saveBufferToFile("dump.bin");

    // Read back:
    gtsam::Values v2;
    {
        auto arch = mrpt::serialization::archiveFrom(buf);
        arch >> v2;
    }

    // Expect equality:
    if (!v.equals(v2))
    {
        v.print("Original values:");
        v2.print("Read-back values:");
        THROW_EXCEPTION("Not identical objects after deserialization");
    }
}

// --------------
static void testSerializeFactorGraph(size_t n)
{
    using namespace gtsam2mrpt_serial;  // expose the << & >> operators;

    const gtsam::NonlinearFactorGraph fg = createTestGraph(n);

    // save to binary stream:
    // (Replace this with CFileGZOutputStream to save to a real file)
    mrpt::io::CMemoryStream buf;
    {
        auto arch = mrpt::serialization::archiveFrom(buf);
        arch << fg;
    }
    buf.Seek(0);

    // Debug:
    // buf.saveBufferToFile("dump.bin");

    // Read back:
    gtsam::NonlinearFactorGraph fg2;
    {
        auto arch = mrpt::serialization::archiveFrom(buf);
        arch >> fg2;
    }

    // Expect equality:
    if (!fg.equals(fg2))
    {
        fg.print("Original FG:");
        fg2.print("Read-back FG:");
        THROW_EXCEPTION("Not identical objects after deserialization");
    }
}

static mrpt::system::CTimeLogger profiler;

static void testSerializeProfiler(size_t n)
{
    using namespace std::string_literals;
    using namespace gtsam2mrpt_serial;  // expose the << & >> operators;

    const size_t numReps = std::max<size_t>(5, 1000 / n);

    const gtsam::Values               v  = createTestValues(n);
    const gtsam::NonlinearFactorGraph fg = createTestGraph(n);

    const auto sProfPost = mrpt::format(
        "%05u_v_%05u_f", static_cast<unsigned int>(v.size()),
        static_cast<unsigned int>(fg.size()));

    const auto sProfSer   = "serialize_"s + sProfPost;
    const auto sProfDeser = "deserialize_"s + sProfPost;

    const auto sProfSerBoost   = "serialize_"s + sProfPost + "_boost"s;
    const auto sProfDeserBoost = "deserialize_"s + sProfPost + "_boost"s;

    for (size_t idx = 0; idx < numReps; idx++)
    {
        // -------------------------------------
        // First use our serialization
        // -------------------------------------
        // save values to binary stream:
        mrpt::io::CMemoryStream buf;
        {
            auto arch = mrpt::serialization::archiveFrom(buf);

            auto tle = mrpt::system::CTimeLoggerEntry(profiler, sProfSer);
            arch << v << fg;
        }
        buf.Seek(0);

        // Read back:
        gtsam::Values               v2;
        gtsam::NonlinearFactorGraph fg2;
        {
            auto arch = mrpt::serialization::archiveFrom(buf);
            auto tle  = mrpt::system::CTimeLoggerEntry(profiler, sProfDeser);
            arch >> v2 >> fg2;
        }

        // Expect equality:
        ASSERT_(v.equals(v2));
        ASSERT_(fg.equals(fg2));

        // -------------------------------------
        // Compare to Boost serialization:
        // -------------------------------------
        std::stringstream binBuf;
        {
            auto tle = mrpt::system::CTimeLoggerEntry(profiler, sProfSerBoost);

            gtsam::serializeToBinaryStream(v, binBuf);
            gtsam::serializeToBinaryStream(fg, binBuf);
        }
        // Read back:
        gtsam::Values               v3;
        gtsam::NonlinearFactorGraph fg3;
        binBuf.seekg(0);
        {
            auto tle =
                mrpt::system::CTimeLoggerEntry(profiler, sProfDeserBoost);

            gtsam::deserializeFromBinaryStream(binBuf, v3);
            gtsam::deserializeFromBinaryStream(binBuf, fg3);
        }

        // Expect equality:
        ASSERT_(v.equals(v3));
        ASSERT_(fg.equals(fg3));
    }
}

// --------------
static int failed = 0;

static void tstWrap(const std::string& name, const std::function<void()>& f)
{
    try
    {
        std::cout << "Test: " << name << "...";
        f();
        std::cout << " PASS" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cout << " FAIL:" << std::endl << e.what() << std::endl;
        failed++;
    }
}

int main(int, char**)
{
    using namespace std::string_literals;

    const std::vector<size_t> sizes = {1, 5, 25, 100, 500, 2000};

    for (const auto size : sizes)
    {
        tstWrap(
            "Values N="s + std::to_string(size),
            [=]() { testSerializeValues(size); });
        tstWrap(
            "FactorGraph  N="s + std::to_string(size),
            [=]() { testSerializeFactorGraph(size); });

        tstWrap(
            "Profiler N="s + std::to_string(size),
            [=]() { testSerializeProfiler(size); });
    }

    // profiler.saveToMFile("profiler.m");

    return failed == 0 ? 0 : 1;
}
