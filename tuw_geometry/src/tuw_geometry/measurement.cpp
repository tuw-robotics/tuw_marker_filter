#include <tuw_geometry/measurement.h>

using namespace tuw;

Measurement::Measurement ( Measurement::Type type )
    : type_ ( type ) {};

Measurement::Measurement ( const Measurement &o )
    : type_ ( o.type_ ), stamp_ ( o.stamp_ ), sensor_pose_ ( o.sensor_pose_ ) {
}
Measurement::Type Measurement::getType() const {
    return type_;
}
const std::string Measurement::getTypeName() const {
    switch ( type_ ) {
    case Type::LASER:
        return "LASER";
    case Type::LINE:
        return "LINE";
    case Type::FIDUCIAL:
        return "FIDUCIAL";
    }
    return "NA";
}
const Pose2D& Measurement::getSensorPose() const {
    return sensor_pose_;
}
Pose2D& Measurement::getSensorPose() {
    return sensor_pose_;
}
const boost::posix_time::ptime& Measurement::stamp() const {
    return stamp_;
}
boost::posix_time::ptime& Measurement::stamp() {
    return stamp_;
}
