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
#include <gtsam/nonlinear/Values.h>
#include <mrpt/core/exceptions.h>
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
