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

// test classes:
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Values.h>

// --------------
static void testSerializeValues()
{
    using namespace gtsam2mrpt_serial;  // expose the << & >> operators;

    using gtsam::symbol_shorthand::X;

    gtsam::Values v;

    v.insert(X(0), gtsam::Pose2(1.0, 2.0, 3.0));
    v.insert(X(1), gtsam::Pose3::identity());
    v.insert(X(2), gtsam::Point2(1.0, 2.0));
    v.insert(X(3), gtsam::Point3(10.0, 20.0, -5.0));

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
static int failed = 0;

static void testWrapper(const std::string& name, const std::function<void()>& f)
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
    testWrapper("testSerializeValues", &testSerializeValues);

    return failed == 0 ? 0 : 1;
}
