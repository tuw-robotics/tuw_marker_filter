#ifndef LINESEGMENT2D_H
#define LINESEGMENT2D_H

#include "tuw_geometry/line2d.h"

namespace tuw {

class LineSegment2D; /// Prototype
using LineSegment2DPtr = std::shared_ptr< LineSegment2D > ;
using LineSegment2DConstPtr = std::shared_ptr< LineSegment2D const>;

/**
 * class to represent a 2D line with its endpoints and as equation a*x + b*y + c = 0
 **/
class LineSegment2D : public Line2D {
protected:
    Point2D p0_, p1_;  /// the lines endpoints
public:
    /// constructor
    LineSegment2D();
    /**
     * copy constructor
     * @param l LineSegment2D
     **/
    LineSegment2D ( const LineSegment2D &l );
    /**
     * constructor to create a line from points
     * @param p0
     * @param p1
     **/
    LineSegment2D ( const Point2D &p0, const Point2D &p1 );
    /**
     * constructor to create a line from points
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     **/
    LineSegment2D ( const double &x0, const double &y0, const double &x1, const double &y1 );
    /**
     * @return startpoint x
     **/
    const double &x0() const;
    /**
     * @return startpoint y
     **/
    const double &y0() const;
    /**
     * @return endpoint x
     **/
    const double &x1() const;
    /**
     * @return endpoint y
     **/
    const double &y1() const;
    /**
     * orientation of the line in space
     * @retun angle between -PI and PI
     **/
    double angle() const;
    /**
     * @return startpoint
     **/
    const Point2D &p0() const;
    /**
     * @return endpoint
     **/
    const Point2D &p1() const;
    /**
     * @return center point between p0 and p1
     **/
    Point2D pc() const;
    /**
     * @return the line function of the base class
     **/
    const Line2D &line() const;
    /**
     * @return length of the line
     **/
    const double length() const;
    /**
     * comparison operator
     * @return true on equal
     **/
    bool operator == ( const LineSegment2D& o ) const;
    /**
     * sets a line from points
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     **/
    LineSegment2D& set ( const double &x0, const double &y0, const double &x1, const double &y1 );
    /**
     * sets a line from points
     * @param p0
     * @param p1
     **/
    LineSegment2D& set ( const Point2D &p0, const Point2D &p1 );
    /** computes distance to line segment
     * @see http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     * @param p point
     * @param dx vector to point x
     * @param dx vector to point y
     * @return distance to line between the segment endpoints or the distance to the nearest endpoints 
     **/
    double distanceTo ( const Point2D &p, double &dx, double &dy ) const;
    /** computes distance to line segment
     * @see http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     * @return distance to line between the segment endpoints or the distance to the nearest endpoints 
     **/
    double distanceTo ( const Point2D &p ) const;
    friend std::ostream &operator << ( std::ostream &os, const LineSegment2D &o ) {
        os << "[[" << o.x0() <<  ", " << o.y0() << "], [" << o.x1() << ", " << o.y1() << "] ]";
        return os;
    };
};

};
#endif // LINESEGMENT2D_H
