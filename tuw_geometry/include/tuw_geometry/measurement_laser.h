#ifndef MEASUREMENT_LASER_H
#define MEASUREMENT_LASER_H

#include <vector>
#include <tuw_geometry/point2d.h>
#include <tuw_geometry/measurement.h>

namespace tuw {

class MeasurementLaser;  /// Prototype
using MeasurementLaserPtr =  std::shared_ptr< MeasurementLaser >;
using MeasurementLaserConstPtr = std::shared_ptr< MeasurementLaser const>; 

/**
 * class to represent laser measurements
 **/
class MeasurementLaser : public Measurement{
public:
    /**
    * class to represent a single laser beam measurements with redundant information
    **/
    struct Beam {
        bool valid;
        double length;
        double angle;
        Point2D end_point;
    };
    /**
     * constructor
     * @param distinguish extended classes in base versions
     **/
    MeasurementLaser();
    /**
     * copy constructor
     * @param o source
     **/
    MeasurementLaser(const MeasurementLaser &o);
    /**
     * resizes the vector holding the beam measurements
     * @param size
     **/
    void resize ( size_t n );
    /**
     * returns the max range measurement
     * @return possible max measurement
     **/
    double &range_max();
    /**
     * returns the max range measurement
     * @return possible max measurement
     **/
    const double &range_max() const;
    /**
     * returns the min range measurement
     * @return possible min measurement
     **/
    double &range_min();
    /**
     * returns the min range measurement
     * @return possible min measurement
     **/
    const double &range_min() const;
    /**
     * @return true on empty
     **/
    bool empty() const;
    /**
     * @return number of beams
     **/
    size_t size() const;
    /**
     * @param i idx
     * @return a single beam
     **/
    Beam& operator[] ( int i );
    /**
     * @param i idx
     * @return a single beam
     **/
    const Beam& operator[] ( int i ) const;
private:
    double range_max_;
    double range_min_;
    std::vector<Beam> beam_;
};
};

#endif 
