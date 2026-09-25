/* +------------------------------------------------------------------------+
   |                     gtsam2mrpt_serial library                          |
   |                                                                        |
   | Copyright (c) 2022-2026, Jose Luis Blanco Claraco and contributors     |
   | Released under 3-clause BSD license                                    |
   | SPDX-License-Identifier: BSD-3-Clause                                  |
   +------------------------------------------------------------------------+ */

#include <gtsam/base/serialization.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam2mrpt_serial/gtsam_detect_version.h>
#include <gtsam2mrpt_serial/serialize.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTimeLogger.h>

#include <functional>
#include <iostream>
#include <limits>

#if GTSAM_USES_BOOST
#include "boost-exports.h"
#endif

#include "sampleData.h"

// --------------
static void testSerializeValues(size_t n)
{
    using namespace gtsam2mrpt_serial;  // expose the << & >> operators;

    const gtsam::Values v = createTestValues(n);

    // save values to binary stream:
    // (Replace this with CCompressedOutputStream to save to a real file)
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
    // (Replace this with CCompressedOutputStream to save to a real file)
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
// Serializes and reads back a graph, which must be identical to the input.
static void roundTripGraph(const gtsam::NonlinearFactorGraph& fg)
{
    using namespace gtsam2mrpt_serial;  // expose the << & >> operators;

    mrpt::io::CMemoryStream buf;
    {
        auto arch = mrpt::serialization::archiveFrom(buf);
        arch << fg;
    }
    buf.Seek(0);

    gtsam::NonlinearFactorGraph fg2;
    {
        auto arch = mrpt::serialization::archiveFrom(buf);
        arch >> fg2;
    }

    if (!fg.equals(fg2))
    {
        fg.print("Original FG:");
        fg2.print("Read-back FG:");
        THROW_EXCEPTION("Not identical objects after deserialization");
    }
}

// Returns true if \a f throws an exception.
static bool throws(const std::function<void()>& f)
{
    try
    {
        f();
    }
    catch (const std::exception&)
    {
        return true;
    }
    return false;
}

// --------------
static void testNoiseModels()
{
    using namespace gtsam;
    using namespace gtsam::noiseModel;

    const Pose2 p(1.0, 2.0, 0.3);

    Matrix33 R;
    R << 2.0, 0.1, 0.2,  //
        0.0, 3.0, 0.3,  //
        0.0, 0.0, 4.0;

    const std::vector<SharedNoiseModel> models = {
        Unit::Create(3),
        Isotropic::Sigma(3, 0.1),
        Diagonal::Sigmas(Vector3(0.1, 0.2, 0.3)),
        Gaussian::SqrtInformation(R),
        Constrained::All(3),
        Constrained::MixedSigmas(Vector3(0.0, 0.5, 0.5)),
        Constrained::MixedSigmas(
            Vector3(100.0, 200.0, 300.0), Vector3(0.0, 0.0, 0.2)),
        SharedNoiseModel(),  // no noise model
    };

    for (const auto& m : models)
    {
        NonlinearFactorGraph fg;
        fg.emplace_shared<PriorFactor<Pose2>>(1, p, m);
        fg.emplace_shared<BetweenFactor<Pose2>>(1, 2, p, m);
        roundTripGraph(fg);
    }
}

// --------------
static void testRobustNoiseModels()
{
    using namespace gtsam;
    using namespace gtsam::noiseModel;

    const auto schemes = {mEstimator::Base::Scalar, mEstimator::Base::Block};

    for (const auto scheme : schemes)
    {
        const std::vector<mEstimator::Base::shared_ptr> estimators = {
            mEstimator::Null::Create(),
            mEstimator::Fair::Create(1.1, scheme),
            mEstimator::Huber::Create(1.2, scheme),
            mEstimator::Cauchy::Create(1.3, scheme),
            mEstimator::Tukey::Create(1.4, scheme),
            mEstimator::Welsch::Create(1.5, scheme),
            mEstimator::GemanMcClure::Create(1.6, scheme),
            mEstimator::DCS::Create(1.7, scheme),
            mEstimator::L2WithDeadZone::Create(1.8, scheme),
        };

        for (const auto& e : estimators)
        {
            NonlinearFactorGraph fg;
            fg.emplace_shared<PriorFactor<Pose3>>(
                1, Pose3::Identity(),
                Robust::Create(e, Isotropic::Sigma(6, 0.1)));
            roundTripGraph(fg);
        }
    }
}

// --------------
// An m-estimator unknown to the library.
class UnsupportedEstimator : public gtsam::noiseModel::mEstimator::Base
{
   public:
    double weight(double) const override { return 1.0; }
    double loss(double d) const override { return 0.5 * d * d; }
    void   print(const std::string&) const override {}
    bool   equals(const Base&, double) const override { return false; }
};

static void testUnsupportedEstimatorThrows()
{
    using namespace gtsam;
    using namespace gtsam2mrpt_serial;

    NonlinearFactorGraph fg;
    fg.emplace_shared<PriorFactor<Pose2>>(
        1, Pose2(),
        noiseModel::Robust::Create(
            noiseModel::mEstimator::Base::shared_ptr(
                new UnsupportedEstimator()),
            noiseModel::Unit::Create(3)));

    mrpt::io::CMemoryStream buf;
    auto                    arch = mrpt::serialization::archiveFrom(buf);
    ASSERT_(throws([&]() { arch << fg; }));
}

// --------------
// A factor with an arbitrary number of keys.
class ManyKeysFactor : public gtsam::NonlinearFactor
{
   public:
    explicit ManyKeysFactor(const gtsam::KeyVector& keys)
        : gtsam::NonlinearFactor(keys)
    {
    }
    double error(const gtsam::Values&) const override { return 0; }
    size_t dim() const override { return 0; }
    gtsam::GaussianFactor::shared_ptr linearize(
        const gtsam::Values&) const override
    {
        return {};
    }
};

static void testTooManyKeysThrows()
{
    using namespace gtsam2mrpt_serial;

    gtsam::KeyVector keys(std::numeric_limits<uint16_t>::max() + 1UL);
    for (size_t i = 0; i < keys.size(); i++) { keys[i] = i; }
    const ManyKeysFactor f(keys);

    mrpt::io::CMemoryStream buf;
    auto                    arch = mrpt::serialization::archiveFrom(buf);
    ASSERT_(throws([&]() { arch << f; }));
}

// --------------
// Files written by older versions store Constrained models without sigmas.
static void testReadLegacyConstrained()
{
    using namespace gtsam;
    using namespace gtsam2mrpt_serial;

    const Vector3 mu(100.0, 200.0, 300.0);
    const Pose2   p(1.0, 2.0, 0.3);

    mrpt::io::CMemoryStream buf;
    {
        auto arch = mrpt::serialization::archiveFrom(buf);
        // Keys:
        arch.WriteAs<uint16_t>(1);
        arch << Key(5);
        arch.WriteAs<std::string>("PriorFactor<Pose2>");
        // Noise model:
        arch.WriteAs<bool>(true);
        arch.WriteAs<uint16_t>(3);
        arch.WriteAs<std::string>("Constrained");
        arch << mrpt::math::CMatrixD(Matrix(mu));
        // Value:
        arch.WriteAs<std::string>("Pose2");
        arch << p.x() << p.y() << p.theta();
    }
    buf.Seek(0);

    auto arch = mrpt::serialization::archiveFrom(buf);
    const NonlinearFactor::shared_ptr f(deserialize_factor(arch));

    const PriorFactor<Pose2> expected(
        5, p, noiseModel::Constrained::All(3, mu));
    ASSERT_(expected.equals(*f));
}

#if GTSAM_USES_BOOST

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

    const auto sProfSer    = "serialize_"s + sProfPost;
    const auto sProfDeser  = "deserialize_"s + sProfPost;
    const auto sProfSerLen = "serialize_"s + sProfPost + "_bytes"s;

    const auto sProfSerBoost   = "serialize_"s + sProfPost + "_boost"s;
    const auto sProfDeserBoost = "deserialize_"s + sProfPost + "_boost"s;
    const auto sProfSerBoostLen =
        "serialize_"s + sProfPost + "_boost"s + "_bytes"s;

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

            tle.stop();
            profiler.registerUserMeasure(sProfSerLen, buf.getTotalBytesCount());
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

            tle.stop();
            profiler.registerUserMeasure(sProfSerBoostLen, binBuf.str().size());
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
#endif

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

    tstWrap("NoiseModels", testNoiseModels);
    tstWrap("RobustNoiseModels", testRobustNoiseModels);
    tstWrap("UnsupportedEstimatorThrows", testUnsupportedEstimatorThrows);
    tstWrap("TooManyKeysThrows", testTooManyKeysThrows);
    tstWrap("ReadLegacyConstrained", testReadLegacyConstrained);

    const std::vector<size_t> sizes = {1, 5, 25, 100, 500, 2000};

    for (const auto size : sizes)
    {
        tstWrap(
            "Values N="s + std::to_string(size),
            [=]() { testSerializeValues(size); });
        tstWrap(
            "FactorGraph  N="s + std::to_string(size),
            [=]() { testSerializeFactorGraph(size); });

#if GTSAM_USES_BOOST
        tstWrap(
            "Profiler N="s + std::to_string(size),
            [=]() { testSerializeProfiler(size); });
#endif
    }

    // profiler.saveToMFile("profiler.m");

    return failed == 0 ? 0 : 1;
}
