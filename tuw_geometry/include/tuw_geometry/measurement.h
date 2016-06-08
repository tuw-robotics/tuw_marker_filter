#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <memory>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <tuw_geometry/pose2d.h>

namespace tuw {

class Measurement; /// Prototype
using MeasurementPtr = std::shared_ptr< Measurement >;
using MeasurementConstPtr = std::shared_ptr< Measurement const>;
/**
 * class to represent sensor measurements
 **/
class Measurement {
public:
  /**
   * to distinguish extended virtual classes
   **/
    enum class Type {
        LASER = 0,
        LINE = 1,
        FIDUCIAL = 2
    };
    /**
     * constructor
     * @param distinguish extended classes in base versions
     **/
    Measurement ( Type type );
    /**
     * copy constructor
     * @param o source
     **/
    Measurement ( const Measurement &o);
    /**
     * to distinguish extended classes in base versions
     * @return type
     **/
    Type getType() const;
    virtual bool empty() const = 0;
    /**
     * returns a human readable type name
     * @return name
     **/
    const std::string getTypeName() const;
    /**
     * sensor pose related to the measurement
     * @return pose
     **/
    const tuw::Pose2D& getSensorPose() const;
    /**
     * sensor pose related to the measurement
     * @return pose
     **/
    tuw::Pose2D& getSensorPose();
    /**
     * transformation related to the measurement
     * @return matrix
     **/
    const boost::posix_time::ptime& stamp() const;
    /**
     * timestamp related to the measurement
     * @return stamp
     **/
    boost::posix_time::ptime& stamp();
private:
    Type type_;
    boost::posix_time::ptime stamp_;
    tuw::Pose2D sensor_pose_;
};
};

#endif
