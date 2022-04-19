/* +------------------------------------------------------------------------+
   |                     gtsam2mrpt_serial library                          |
   |                                                                        |
   | Copyright (c) 2022, Jose Luis Blanco Claraco and contributors          |
   | Released under 3-clause BSD license                                    |
   +------------------------------------------------------------------------+ */

#include <gtsam2mrpt_serial/serialize.h>
//
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/serialization/CArchive.h>

#include <typeinfo>

// ----------------------------------
// Serialize individual values
// ----------------------------------
mrpt::serialization::CArchive& gtsam2mrpt_serial::operator<<(
    mrpt::serialization::CArchive& out, const gtsam::Value& value)
{
    using namespace gtsam;

    if (auto* val = dynamic_cast<const GenericValue<Pose2>*>(&value); val)
    {
        out.WriteAs<std::string>("Pose2");
        const auto& v = val->value();
        out << v.x() << v.y() << v.theta();
    }
    else if (auto* val = dynamic_cast<const GenericValue<Pose3>*>(&value); val)
    {
        out.WriteAs<std::string>("Pose3");
        const auto& v = val->value();
        out << mrpt::gtsam_wrappers::toTPose3D(v);
    }
    else if (auto* val = dynamic_cast<const GenericValue<Point2>*>(&value); val)
    {
        out.WriteAs<std::string>("Point2");
        const auto& v = val->value();
        out << v.x() << v.y();
    }
    else if (auto* val = dynamic_cast<const GenericValue<Point3>*>(&value); val)
    {
        out.WriteAs<std::string>("Point3");
        const auto& v = val->value();
        out << v.x() << v.y() << v.z();
    }
    else
    {
        std::cerr << "Serialization not implemented for this gtsam::Value:\n";
        value.print();
        THROW_EXCEPTION(
            "Serialization not implemented, see error message above for type "
            "details.");
    }

    return out;
}

// ----------------------------------
// De-serialize one individual value
// ----------------------------------
void gtsam2mrpt_serial::deserialize_and_insert(
    mrpt::serialization::CArchive& in, uint64_t key, gtsam::Values& values)
{
    const auto typeName = in.ReadAs<std::string>();
    if (typeName == "Pose2")
    {
        double x, y, z;
        in >> x >> y >> z;
        gtsam::Pose2 v(x, y, z);
        values.insert(key, v);
    }
    else if (typeName == "Pose3")
    {
        mrpt::math::TPose3D p;
        in >> p;
        gtsam::Pose3 v = mrpt::gtsam_wrappers::toPose3(p);
        values.insert(key, v);
    }
    else if (typeName == "Point2")
    {
        double x, y;
        in >> x >> y;
        gtsam::Point2 v(x, y);
        values.insert(key, v);
    }
    else if (typeName == "Point3")
    {
        double x, y, z;
        in >> x >> y >> z;
        gtsam::Point3 v(x, y, z);
        values.insert(key, v);
    }
    else
    {
        THROW_EXCEPTION_FMT(
            "De-serialization not implemented for gtsam::Value type '%s'",
            typeName.c_str());
    }
}

// ----------------------------------
// Serialize Values
// ----------------------------------
constexpr uint8_t VALUES_SERIAL_VERSION = 0;

mrpt::serialization::CArchive& gtsam2mrpt_serial::operator<<(
    mrpt::serialization::CArchive& out, const gtsam::Values& values)
{
    out.WriteAs<std::string>("Values");
    out << VALUES_SERIAL_VERSION;
    out.WriteAs<uint64_t>(values.size());

    for (const auto& v : values) { out << v.key << v.value; }

    return out;
}

// ----------------------------------
// De-serialize Values
// ----------------------------------
mrpt::serialization::CArchive& gtsam2mrpt_serial::operator>>(
    mrpt::serialization::CArchive& in, gtsam::Values& values)
{
    const auto signature = in.ReadAs<std::string>();
    ASSERT_EQUAL_(signature, "Values");
    ASSERT_EQUAL_(in.ReadAs<uint8_t>(), VALUES_SERIAL_VERSION);

    values = gtsam::Values();

    const auto n = in.ReadAs<uint64_t>();
    for (size_t i = 0; i < n; i++)
    {
        gtsam::Key k;
        in >> k;
        deserialize_and_insert(in, k, values);
    }

    return in;
}

// ----------------------------------
// Serialize FG
// ----------------------------------
constexpr uint8_t FG_SERIAL_VERSION = 0;

mrpt::serialization::CArchive& gtsam2mrpt_serial::operator<<(
    mrpt::serialization::CArchive& out, const gtsam::NonlinearFactorGraph& fg)
{
    out.WriteAs<std::string>("NonlinearFactorGraph");
    out << FG_SERIAL_VERSION;
    out.WriteAs<uint64_t>(fg.size());

    for (const auto& f : fg)
    {
        out.WriteAs<bool>(f.get() != nullptr);
        if (!f) continue;

        out << *f;
    }

    return out;
}

// ----------------------------------
// De-serialize FG
// ----------------------------------
mrpt::serialization::CArchive& gtsam2mrpt_serial::operator>>(
    mrpt::serialization::CArchive& in, gtsam::NonlinearFactorGraph& fg)
{
    const auto signature = in.ReadAs<std::string>();
    ASSERT_EQUAL_(signature, "NonlinearFactorGraph");
    ASSERT_EQUAL_(in.ReadAs<uint8_t>(), FG_SERIAL_VERSION);

    fg = gtsam::NonlinearFactorGraph();

    const auto n = in.ReadAs<uint64_t>();
    fg.resize(n);

    for (size_t i = 0; i < n; i++)
    {
        const bool isNotNull = in.ReadAs<bool>();
        if (!isNotNull)
        {
            fg[i].reset();
            continue;
        }
        fg[i].reset(deserialize_factor(in));
    }

    return in;
}

static void serialize_noise_robust(
    mrpt::serialization::CArchive&                         out,
    const gtsam::noiseModel::mEstimator::Base::shared_ptr& robust)
{
    out.WriteAs<bool>(robust.get() != nullptr);
    if (!robust) return;

    using namespace gtsam;
    using namespace gtsam::noiseModel;

    // Base: ReweightScheme reweight_;
    out << robust->reweight_;

    // Derived:
    if (auto* n = dynamic_cast<const mEstimator::Null*>(robust.get()); n)
    {
        //
        out.WriteAs<std::string>("Null");
    }
    else if (auto* n = dynamic_cast<const mEstimator::Fair*>(robust.get()); n)
    {
        out.WriteAs<std::string>("Fair");
        out << n->c_;
    }
    else if (auto* n = dynamic_cast<const mEstimator::Huber*>(robust.get()); n)
    {
        out.WriteAs<std::string>("Huber");
        out << n->k_;
    }
    else if (auto* n = dynamic_cast<const mEstimator::Cauchy*>(robust.get()); n)
    {
        out.WriteAs<std::string>("Cauchy");
        out << n->k_;
    }
    else if (auto* n = dynamic_cast<const mEstimator::Tukey*>(robust.get()); n)
    {
        out.WriteAs<std::string>("Tukey");
        out << n->c_;
    }
    else if (auto* n = dynamic_cast<const mEstimator::Welsch*>(robust.get()); n)
    {
        out.WriteAs<std::string>("Welsch");
        out << n->c_;
    }
    else if (auto* n =
                 dynamic_cast<const mEstimator::GemanMcClure*>(robust.get());
             n)
    {
        out.WriteAs<std::string>("GemanMcClure");
        out << n->c_;
    }
    else if (auto* n = dynamic_cast<const mEstimator::DCS*>(robust.get()); n)
    {
        out.WriteAs<std::string>("DCS");
        out << n->c_;
    }
    else if (auto* n =
                 dynamic_cast<const mEstimator::L2WithDeadZone*>(robust.get());
             n)
    {
        out.WriteAs<std::string>("L2WithDeadZone");
        out << n->k_;
    }
}

static void serialize_noise_model(
    mrpt::serialization::CArchive& out, const gtsam::SharedNoiseModel& noise)
{
    out.WriteAs<bool>(noise.get() != nullptr);
    if (!noise) return;

    using namespace gtsam;

    // Base data:
    out.WriteAs<uint16_t>(noise->dim());

    // Derived:
    if (auto* n = dynamic_cast<const noiseModel::Gaussian*>(noise.get()); n)
    {
        out.WriteAs<std::string>("Gaussian");
        out << mrpt::math::CMatrixD(n->R());
    }
    else if (auto* n = dynamic_cast<const noiseModel::Diagonal*>(noise.get());
             n)
    {
        out.WriteAs<std::string>("Diagonal");
        out << mrpt::math::CMatrixD(n->sigmas());
    }
    else if (auto* n =
                 dynamic_cast<const noiseModel::Constrained*>(noise.get());
             n)
    {
        out.WriteAs<std::string>("Constrained");
        out << mrpt::math::CMatrixD(n->mu());
    }
    else if (auto* n = dynamic_cast<const noiseModel::Isotropic*>(noise.get());
             n)
    {
        out.WriteAs<std::string>("Isotropic");
        out << n->sigma();
    }
    else if (auto* n = dynamic_cast<const noiseModel::Unit*>(noise.get()); n)
    {
        out.WriteAs<std::string>("Unit");
    }
    else if (auto* n = dynamic_cast<const noiseModel::Robust*>(noise.get()); n)
    {
        out.WriteAs<std::string>("Robust");
        serialize_noise_robust(out, n->robust());
        serialize_noise_model(out, n->noise());
    }
    else
    {
        std::cerr << "Unknown noiseModel found whie serializing:\n";
        noise->print();
        THROW_EXCEPTION("Unknown noiseModel found whie serializing.");
    }
}

// ----------------------------------
// Serialize individual Factors
// ----------------------------------
mrpt::serialization::CArchive& gtsam2mrpt_serial::operator<<(
    mrpt::serialization::CArchive& out, const gtsam::NonlinearFactor& factor)
{
    using namespace gtsam;

    // Keys:
    out.WriteAs<uint16_t>(factor.keys().size());
    for (const auto& k : factor.keys()) out << k;

#define SERIALIZE_PRIOR_FACTOR(TYPE__)                                       \
    else if (auto* f = dynamic_cast<const PriorFactor<TYPE__>*>(&factor); f) \
    {                                                                        \
        out.WriteAs<std::string>("PriorFactor<" #TYPE__ ">");                \
        serialize_noise_model(out, f->noiseModel());                         \
        out << GenericValue<TYPE__>(f->prior());                             \
    }

#define SERIALIZE_BETWEEN_FACTOR(TYPE__)                                       \
    else if (auto* f = dynamic_cast<const BetweenFactor<TYPE__>*>(&factor); f) \
    {                                                                          \
        out.WriteAs<std::string>("BetweenFactor<" #TYPE__ ">");                \
        serialize_noise_model(out, f->noiseModel());                           \
        out << GenericValue<TYPE__>(f->measured());                            \
    }

    // Factor itself:
    if (0) {}
    //
    SERIALIZE_PRIOR_FACTOR(Point2)
    SERIALIZE_PRIOR_FACTOR(Point3)
    SERIALIZE_PRIOR_FACTOR(Pose2)
    SERIALIZE_PRIOR_FACTOR(Pose3)
    //
    SERIALIZE_BETWEEN_FACTOR(Point2)
    SERIALIZE_BETWEEN_FACTOR(Point3)
    SERIALIZE_BETWEEN_FACTOR(Pose2)
    SERIALIZE_BETWEEN_FACTOR(Pose3)
    //
    else
    {
        std::cerr << "Serialization not implemented for this "
                     "gtsam::NonlinearFactor:\n";
        factor.print();
        THROW_EXCEPTION(
            "Serialization not implemented, see error message above for type "
            "details.");
    }

    return out;
}

// ----------------------------------
// De-serialize one individual value
// ----------------------------------
gtsam::NonlinearFactor* gtsam2mrpt_serial::deserialize_factor(
    mrpt::serialization::CArchive& in)
{
    //    const auto typeName = in.ReadAs<std::string>();

    return nullptr;
}
