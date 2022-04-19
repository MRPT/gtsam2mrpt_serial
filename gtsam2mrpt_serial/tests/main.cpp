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

// Geometry:
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

// Nonlinear high level:
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Values.h>

// Factors:
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

/*
 *  typedef PriorFactor<Point2>                 PriorFactorPoint2;
 *  typedef PriorFactor<Pose3>                  PriorFactorPose3;
 *  typedef BetweenFactor<Point2> BetweenFactorPoint2;
 *  typedef BetweenFactor<Pose3>  BetweenFactorPose3;
 *  typedef NonlinearEquality<Point2>           NonlinearEqualityPoint2;
 *
 */

// --------------
static gtsam::Values createTestValues()
{
    using gtsam::symbol_shorthand::X;

    gtsam::Values v;

    v.insert(X(0), gtsam::Pose2(1.0, 2.0, 3.0));
    v.insert(X(1), gtsam::Pose3::identity());
    v.insert(X(2), gtsam::Point2(1.0, 2.0));
    v.insert(X(3), gtsam::Point3(10.0, 20.0, -5.0));

    return v;
}

// --------------
static gtsam::NonlinearFactorGraph createTestGraph()
{
    using gtsam::symbol_shorthand::X;
    using namespace gtsam;

    gtsam::NonlinearFactorGraph fg;

    SharedNoiseModel noise2 = noiseModel::Isotropic::Sigma(2, 0.1);
    SharedNoiseModel noise3 = noiseModel::Isotropic::Sigma(3, 0.1);
    SharedNoiseModel noise6 = noiseModel::Isotropic::Sigma(6, 0.1);

    SharedNoiseModel robust3 = noiseModel::Robust::Create(
        noiseModel::mEstimator::Huber::Create(
            10.0, noiseModel::mEstimator::Huber::Scalar),
        noiseModel::Unit::Create(3));
    SharedNoiseModel robust6 = noiseModel::Robust::Create(
        noiseModel::mEstimator::Huber::Create(
            10.0, noiseModel::mEstimator::Huber::Scalar),
        noiseModel::Unit::Create(6));

    fg.addPrior<Point2>(X(0), Point2(1.0, 2.0));
    fg.at(0).reset();  // leave one nullptr factor to test its serialization.

    fg.addPrior<Point2>(X(0), Point2(1.0, 2.0));
    fg.addPrior<Point2>(X(1), Point2(1.0, 2.0), noise2);
    fg.addPrior<Point3>(X(2), Point3(1.0, 2.0, 3.0), noise3);

    fg.addPrior<Pose2>(X(3), Pose2(6.0, 5.0, -1.0), noise3);
    fg.addPrior<Pose2>(X(3), Pose2(6.0, 5.0, -1.0), robust3);

    fg.addPrior<Pose3>(X(4), Pose3::identity(), noise6);
    fg.addPrior<Pose3>(X(4), Pose3::identity(), robust6);

    fg.emplace_shared<BetweenFactor<Pose2>>(
        X(10), X(11), Pose2::identity(), noise3);
    fg.emplace_shared<BetweenFactor<Pose2>>(
        X(10), X(11), Pose2::identity(), robust3);

    fg.emplace_shared<BetweenFactor<Pose3>>(
        X(20), X(21), Pose3::identity(), noise6);
    fg.emplace_shared<BetweenFactor<Pose3>>(
        X(20), X(21), Pose3::identity(), robust6);

    return fg;
}

// --------------
static void testSerializeValues()
{
    using namespace gtsam2mrpt_serial;  // expose the << & >> operators;

    const gtsam::Values v = createTestValues();

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
static void testSerializeFactorGraph()
{
    using namespace gtsam2mrpt_serial;  // expose the << & >> operators;

    const gtsam::NonlinearFactorGraph fg = createTestGraph();

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
    testWrapper("testSerializeFactorGraph", &testSerializeFactorGraph);

    return failed == 0 ? 0 : 1;
}
