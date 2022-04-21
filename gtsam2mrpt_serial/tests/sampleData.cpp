/* +------------------------------------------------------------------------+
   |                     gtsam2mrpt_serial library                          |
   |                                                                        |
   | Copyright (c) 2022, Jose Luis Blanco Claraco and contributors          |
   | Released under 3-clause BSD license                                    |
   +------------------------------------------------------------------------+ */

#include "sampleData.h"

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
#include <mrpt/random/RandomGenerators.h>

static double rnd()
{
    return mrpt::random::getRandomGenerator().drawUniform(-1.0, 1.0);
}

gtsam::Values createTestValues(size_t count)
{
    using gtsam::symbol_shorthand::A;
    using gtsam::symbol_shorthand::B;
    using gtsam::symbol_shorthand::C;
    using gtsam::symbol_shorthand::D;

    const auto rot1 = gtsam::Rot3::RzRyRx(0.3, 0.1, -0.5);
    const auto rot2 = gtsam::Rot3::RzRyRx(-0.2, -0.7, 1.8);

    gtsam::Values v;

    for (size_t i = 0; i < count; i++)
        v.insert(A(i), gtsam::Pose2(rnd(), rnd(), rnd()));

    for (size_t i = 0; i < count; i++)
    {
        v.insert(B(3 * i + 0), gtsam::Pose3::identity());
        v.insert(B(3 * i + 1), gtsam::Rot3::RzRyRx(rnd(), rnd(), rnd()));
        v.insert(
            B(3 * i + 2), gtsam::Pose3(
                              gtsam::Rot3::RzRyRx(rnd(), rnd(), rnd()),
                              {rnd(), rnd(), rnd()}));
    }

    for (size_t i = 0; i < count; i++)  //
        v.insert(C(i), gtsam::Point2(rnd(), rnd()));

    for (size_t i = 0; i < count; i++)
        v.insert(D(i), gtsam::Point3(rnd(), rnd(), rnd()));

    return v;
}

gtsam::NonlinearFactorGraph createTestGraph(size_t count)
{
    using namespace gtsam::symbol_shorthand;
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

    for (size_t i = 0; i < count; i++)
    {
        fg.addPrior<Point2>(A(i), Point2(rnd(), rnd()));
        fg.addPrior<Point2>(B(i), Point2(rnd(), rnd()), noise2);
        fg.addPrior<Point3>(C(i), Point3(rnd(), rnd(), rnd()), noise3);

        fg.addPrior<Pose2>(D(i), Pose2(rnd(), rnd(), rnd()), noise3);
        fg.addPrior<Pose2>(E(i), Pose2(rnd(), rnd(), rnd()), robust3);

        fg.addPrior<Pose3>(F(i), Pose3::identity(), noise6);
        fg.addPrior<Pose3>(G(i), Pose3::identity(), robust6);
        fg.addPrior<Pose3>(
            H(i),
            gtsam::Pose3(
                gtsam::Rot3::RzRyRx(rnd(), rnd(), rnd()),
                {rnd(), rnd(), rnd()}),
            noise6);
    }

    for (size_t i = 0; i < count; i++)
    {
        fg.emplace_shared<BetweenFactor<Pose2>>(
            A(2 * i), A(2 * i + 1), Pose2::identity(), noise3);
        fg.emplace_shared<BetweenFactor<Pose2>>(
            B(2 * i), B(2 * i + 1), Pose2::identity(), robust3);

        fg.emplace_shared<BetweenFactor<Pose3>>(
            C(2 * i), C(2 * i + 1), Pose3::identity(), noise6);
        fg.emplace_shared<BetweenFactor<Pose3>>(
            D(2 * i), D(2 * i + 1),
            gtsam::Pose3(
                gtsam::Rot3::RzRyRx(rnd(), rnd(), rnd()),
                {rnd(), rnd(), rnd()}),
            robust6);
    }

    return fg;
}
