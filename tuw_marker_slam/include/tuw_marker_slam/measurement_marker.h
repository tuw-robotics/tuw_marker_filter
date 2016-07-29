#ifndef MEASUREMENT_MARKER_H
#define MEASUREMENT_MARKER_H

#include <vector>
#include <tuw_geometry/pose2d.h>
#include <tuw_geometry/measurement.h>

namespace tuw {

class MeasurementMarker;  /// Prototype
using MeasurementMarkerPtr =  std::shared_ptr< MeasurementMarker >;
using MeasurementMarkerConstPtr = std::shared_ptr< MeasurementMarker const >; 

/**
 * class to represent marker measurements
 **/
class MeasurementMarker : public Measurement{
public:
    /**
    * class to represent a single marker with redundant information
    **/
    struct Marker {
        std::vector<int> ids;
        std::vector<double> ids_confidence;

        double length;
        double angle;
        double orientation;
        Pose2D pose;
    };
    /**
     * constructor
     * @param distinguish extended classes in base versions
     **/
    MeasurementMarker();
    /**
     * copy constructor
     * @param o source
     **/
    MeasurementMarker(const MeasurementMarker &o);
    /**
     * resizes the vector holding the marker measurements
     * @param size
     **/
    void resize ( size_t n );
    /**
     * Clears the vector holding the marker measurements
     **/
    void clear();
    /**
     * returns the min angle measurement
     * @return possible min measurement
     **/
    double &angle_min();
    /**
     * returns the min angle measurement
     * @return possible min measurement
     **/
    const double &angle_min() const;
    /**
     * returns the max angle measurement
     * @return possible max measurement
     **/
    double &angle_max();
    /**
     * returns the max angle measurement
     * @return possible max measurement
     **/
    const double &angle_max() const;
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
     * returns the max id range measurement
     * @return possible max id measurement
     **/
    double &range_max_id();
    /**
     * returns the max id range measurement
     * @return possible max id measurement
     **/
    const double &range_max_id() const;
    /**
     * @return true on empty
     **/
    bool empty() const;
    /**
     * @return number of markers
     **/
    size_t size() const;
    /**
     * @param i idx
     * @return a single marker
     **/
    Marker& operator[] ( int i );
    /**
     * @param i idx
     * @return a single marker
     **/
    const Marker& operator[] ( int i ) const;
    /**
     * @param m marker
     **/
    void push_back ( const MeasurementMarker::Marker m );
private:
    double angle_min_;
    double angle_max_;

    double range_max_;
    double range_min_;
    double range_max_id_;

    std::vector<Marker> markers_;
};
};

#endif 
