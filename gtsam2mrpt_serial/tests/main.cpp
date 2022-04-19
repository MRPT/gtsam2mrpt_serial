/* +------------------------------------------------------------------------+
   |                     gtsam2mrpt_serial library                          |
   |                                                                        |
   | Copyright (c) 2022, Jose Luis Blanco Claraco and contributors          |
   | Released under 3-clause BSD license                                    |
   +------------------------------------------------------------------------+ */

#include <gtsam2mrpt_serial/serialize.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>

#include <functional>
#include <iostream>

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

    const std::vector<size_t> sizes = {1, 10, 1000};

    for (const auto size : sizes)
    {
        tstWrap(
            "Values N="s + std::to_string(size),
            [=]() { testSerializeValues(size); });
        tstWrap(
            "FactorGraph  N="s + std::to_string(size),
            [=]() { testSerializeFactorGraph(size); });
    }
    return failed == 0 ? 0 : 1;
}
