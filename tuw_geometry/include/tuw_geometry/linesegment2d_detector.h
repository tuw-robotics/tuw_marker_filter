#ifndef LINEFILTER_H
#define LINEFILTER_H

#include <memory>
#include "tuw_geometry/linesegment2d.h"

namespace tuw {

class LineSegment2DDetectorParameter;   /// Prototype
using LineSegment2DDetectorParameterPtr = std::shared_ptr< LineSegment2DDetectorParameter >;
using LineSegment2DDetectorParameterConstPtr = std::shared_ptr< LineSegment2DDetectorParameter const>;
/**
 * class to reprecent parameters of a line detection algoithm
 **/
class LineSegment2DDetectorParameter {
public:
    LineSegment2DDetectorParameter()
        : threshold_split_neighbor ( true )
        , threshold_split ( .05 )
        , min_length ( .1 )
        , min_points_per_line ( 20 )
        , min_points_per_unit ( 10 ) {
    }
    bool threshold_split_neighbor;
    double threshold_split;
    double min_length;
    int min_points_per_line;
    int min_points_per_unit;
};

class LineSegment2DDetector;  /// Prototype
using LineSegment2DDetectorPtr =  std::shared_ptr< LineSegment2DDetector >;
using LineSegment2DDetectorConstPtr = std::shared_ptr< LineSegment2DDetector const>;
/**
 * class to detect lines in 2D points base on a split and merge 
 **/
class LineSegment2DDetector {
public:
    /**
    * An exdented line segment reprecentetion for the detection
    **/
    class LineSegment : public LineSegment2D {
    public:
        /// constructor
        LineSegment() : LineSegment2D(), id_ ( 0 ) {};
        void set ( unsigned int idx0, unsigned int idx1, const std::vector<Point2D> &points );
        void updatePoints ( const std::vector<Point2D> &points );
        bool isSupportPoint ( int idx );
        unsigned int nrSupportPoint();
        unsigned int id_;
        unsigned int idx0_, idx1_;
        std::vector<Point2D> points_;
    };
    LineSegment2DDetector();
    void start ( const std::vector<Point2D> &points);
    std::vector<LineSegment2D> &start ( const std::vector<Point2D> &points, std::vector<LineSegment2D> &detected_segments );
    const std::vector<LineSegment> & result (  );
    LineSegment2DDetectorParameter config_;
private:
    void split ( LineSegment &line, const std::vector<Point2D> &points );
    std::vector< std::pair<unsigned int, unsigned int> > connected_measurments_;
    std::vector<LineSegment> segments_;
};

};
#endif // LINE2DFILTER_H
