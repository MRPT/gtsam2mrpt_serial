/* +------------------------------------------------------------------------+
   |                     gtsam2mrpt_serial library                          |
   |                                                                        |
   | Copyright (c) 2022, Jose Luis Blanco Claraco and contributors          |
   | Released under 3-clause BSD license                                    |
   +------------------------------------------------------------------------+ */

#pragma once

#include <gtsam/base/GenericValue.h>  // GTSAM_VALUE_EXPORT
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/expressions.h>

#include <boost/serialization/export.hpp>  // BOOST_CLASS_EXPORT_GUID
#include <boost/serialization/serialization.hpp>

namespace boost
{
namespace serialization
{
template <class Archive, typename Derived>
void serialize(
    Archive& ar, Eigen::EigenBase<Derived>& g, const unsigned int version)
{
    (void)version;
    ar& boost::serialization::make_array(g.derived().data(), g.size());
}
}  // namespace serialization
}  // namespace boost

/* Create GUIDs for Noisemodels */
/* ************************************************************************* */
// clang-format off
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic,"gtsam_noiseModel_Isotropic")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Robust, "gtsam_noiseModel_Robust")

BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Base, "gtsam_noiseModel_mEstimator_Base")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Null,"gtsam_noiseModel_mEstimator_Null")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Fair, "gtsam_noiseModel_mEstimator_Fair")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Huber,"gtsam_noiseModel_mEstimator_Huber")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Cauchy,"gtsam_noiseModel_mEstimator_Cauchy")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Tukey, "gtsam_noiseModel_mEstimator_Tukey")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Welsch, "gtsam_noiseModel_mEstimator_Welsch")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::GemanMcClure, "gtsam_noiseModel_mEstimator_GemanMcClure")

BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel")
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal")
// clang-format on

/* Create GUIDs for geometry */
/* ************************************************************************* */
GTSAM_VALUE_EXPORT(gtsam::Point2)
GTSAM_VALUE_EXPORT(gtsam::Point3)
GTSAM_VALUE_EXPORT(gtsam::Rot2)
GTSAM_VALUE_EXPORT(gtsam::Rot3)
GTSAM_VALUE_EXPORT(gtsam::Pose2)
GTSAM_VALUE_EXPORT(gtsam::Pose3)
GTSAM_VALUE_EXPORT(double)
GTSAM_VALUE_EXPORT(gtsam::Cal3_S2)
GTSAM_VALUE_EXPORT(gtsam::Cal3DS2)

/* Create GUIDs for factors */
/* ************************************************************************* */
// clang-format off
BOOST_CLASS_EXPORT_GUID(gtsam::JacobianFactor, "gtsam::JacobianFactor")

BOOST_CLASS_EXPORT_GUID(gtsam::ExpressionFactor<gtsam::Point3>, "gtsam::ExpressionFactor<gtsam::Point3>")
BOOST_CLASS_EXPORT_GUID(gtsam::ExpressionFactor<gtsam::Rot3>, "gtsam::ExpressionFactor<gtsam::Rot3>")

using ProjectionFactor3D = gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>;
BOOST_CLASS_EXPORT_GUID(ProjectionFactor3D, "gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>")

using ProjectionFactor3Db = gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>;
BOOST_CLASS_EXPORT_GUID(ProjectionFactor3Db, "gtsam::GenericProjectionFactorPose3Point3Cal3_S2")

BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Point2>, "gtsam::BetweenFactor<gtsam::Point2>")
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Point3>, "gtsam::BetweenFactor<gtsam::Point3>")
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Pose2>, "gtsam::BetweenFactor<gtsam::Pose2>")
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Pose3>, "gtsam::BetweenFactor<gtsam::Pose3>")
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Rot3>, "gtsam::BetweenFactor<gtsam::Rot3>")
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Point2>, "gtsam::PriorFactor<gtsam::Point2>")
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Point3>, "gtsam::PriorFactor<gtsam::Point3>")
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Rot3>, "gtsam::PriorFactor<gtsam::Rot3>")
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Rot2>, "gtsam::PriorFactor<gtsam::Rot2>")
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Pose2>, "gtsam::PriorFactor<gtsam::Pose2>")
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Pose3>, "gtsam::PriorFactor<gtsam::Pose3>")

// clang-format on
