#ifndef LINE2D_H
#define LINE2D_H

#include "tuw_geometry/polar2d.h"
#include <cstdio>

namespace tuw {
class Line2D; /// Prototype
using Line2DPtr = std::shared_ptr< Line2D > ;
using Line2DConstPtr = std::shared_ptr< Line2D const>;

/**
 * class to represent a 2D line as equation a*x + b*y + c = 0
 * The line has no endpoints
 **/
class Line2D : public cv::Vec<double,3> {
public:
    /// constructor
    Line2D();
    /**
     * copy constructor
     * @param l equation
     **/
    Line2D ( const Line2D &l );
    /**
     * constructor with optional normalization
     * @param l equation
     * @param normalize normalizes equation on true
     **/
    Line2D ( cv::Vec<double,3> &l, bool normalize = true );
    /**
     * constructor to create a line from points
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     * @param normalize normalizes equation on true
     **/
    Line2D ( const double &x0, const double &y0, const double &x1, const double &y1, bool normalize = true );
    /**
     * constructor to create a line from points
     * @param pt1
     * @param pt2
     * @param normalize normalizes equation on true
     **/
    Line2D ( const Point2D &pt1, const Point2D &pt2, bool normalize = true );
    /**
     * @return the first equation component for x
     **/
    double &a();
    /**
     * @return the first equation component for x
     **/
    const double &a() const;
    /**
     * @return the second equation component for y
     **/
    double &b();
    /**
     * @return the second equation component for y
     **/
    const double &b() const;
    /**
     * @return the third equation component 
     **/
    double &c();
    /**
     * @return the third equation component 
     **/
    const double &c() const;
    /**
     * normalizes the equation to a*a + b*b = 1
     **/
    void normalize();
    /**
     * computes the distance to a point
     * @param x
     * @param y
     * @returns the minimal distance to a point
     * @pre normalize
     **/
    double distanceTo ( const double &x, const double &y )  const;
    /**
     * computes the distance to a point
     * @param p
     * @returns the minimal distance to a point
     * @pre normalize
     **/
    double distanceTo ( const Point2D &p )  const;
    /**
     * computes a point on the line with the shortest distance to the point given
     * @param x
     * @param y
     * @returns point on line 
     * @pre normalize
     **/
    Point2D pointOnLine ( const double &x, const double &y ) const;
    /**
     * computes a point on the line with the shortest distance to the point given
     * @param p
     * @returns point on line 
     * @pre normalize
     **/
    Point2D pointOnLine ( const Point2D &p ) const;
    /**
     * computes the intersection point of two lines
     * @param l
     * @returns point on line 
     * @pre normalize
     **/
    Point2D intersection ( const Line2D &l ) const;
    /**
     * restuns the normal vector to a line 
     * @return vector
     **/
    cv::Vec<double,2> normal() const;
    /**
     * constructor to create a line from points
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     * @param normalize normalizes equation on true
     * @return ref to this
     **/
    Line2D &set ( const double &x0, const double &y0, const double &x1, const double &y1, bool normalize = true );
    /**
     * constructor to create a line from points
     * @param p0
     * @param p1
     * @param normalize normalizes equation on true
     * @return ref to this
     **/
    Line2D &set ( const Point2D &p0, const Point2D &p1, bool normalize = true );
    /**
     * returns the corresponding opencv vector
     * @return cv vector
     **/
    cv::Vec<double,3> &cv ();
    /**
     * returns the corresponding opencv vector
     * @return cv vector
     **/
    const cv::Vec<double,3> &cv () const;
    /**
     * converts the line into polar coordinates
     * @return Polar2D
     **/
    Polar2D toPolar() const;
    friend std::ostream &operator << ( std::ostream &os, const Line2D &o ) {
        os << "[" << o.a() <<  ", " << o.b() << ", " << o.c() << "]";
        return os;
    };

};

};
#endif // LINE2D_H
