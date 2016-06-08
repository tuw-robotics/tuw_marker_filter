#ifndef POINT2D_H
#define POINT2D_H

#include <memory>
#include <opencv2/core/core.hpp>

namespace tuw {
typedef cv::Matx<double, 3, 3> Tf2D;

/**
 * normalizes an angle between min_angle and max_angle but max_angle - min_angle >= 2PI
 * @param angle to normalize
 * @param min_angle
 * @param max_angle
 * @return normalized angle
 **/
inline double angle_normalize (double angle, double min_angle = -M_PI, double max_angle = +M_PI) {
    while ( angle > max_angle ) angle -= ( 2.*M_PI );
    while ( angle < min_angle ) angle += ( 2.*M_PI );
    
    return angle;
}
/**
 * computes the angle difference between two angles by taking into account the circular space
 * @param alpha0
 * @param angle1
 * @return difference
 **/
inline double angle_difference(double alpha0, double angle1){
  return atan2(sin(alpha0-angle1), cos(alpha0-angle1));
}


class Point2D;  /// Prototype
using Point2DPtr = std::shared_ptr< Point2D >;
using Point2DConstPtr = std::shared_ptr< Point2D const>;

/**
 * class to represent a point using homogeneous coordinates [x, y, 1]
 **/
class Point2D : public cv::Vec<double, 3 > {
public:
    /**
     * constructor
     **/
    Point2D(); 
    /**
     * copy constructor
     * @param p source
     **/
    Point2D ( const Point2D &p );
    /**
     * constructor
     * @param p source
     **/
    Point2D ( const cv::Point &p );
    /**
     * constructor
     * @param x
     * @param y
     **/
    Point2D ( double x, double y );
    /**
     * constructor
     * @param x
     * @param y
     * @param h
     **/
    Point2D ( double x, double y, double h );
    template<typename T> Point2D ( const cv::Vec<T,3> &p ) : cv::Vec<double,3> ( p ) {};
    template<typename T> Point2D ( const cv::Vec<T,2> &p ) : cv::Vec<double,3> ( p ) {};
    template<typename T> Point2D ( const cv::Point_<T> &p ) : cv::Vec<double,3> ( p.x,p.y,1. ) {};

    /**
     * sets values
     * @param x
     * @param y
     * @return this reference
     **/
    Point2D &set ( double x, double y );
    /**
     * sets values
     * @param x
     * @param y
     * @return this reference
     **/
    Point2D &set ( double x, double y, double h );
    /**
     * translational x component
     * @return x component
     **/
    const double &x () const;
    /**
     * translational x component
     * @return x component
     **/
    double &x ();
    /**
     * translational y component
     * @return y component
     **/
    const double &y () const;
    /**
     * translational y component
     * @return y component
     **/
    double &y ();
    /**
     * homogeneous component
     * @return rotation
     **/
    const double &h () const;
    /**
     * homogeneous component
     * @return rotation
     **/
    double &h ();
    /** 
     * vector without homogeneous component
     * @return state vector
     **/
    cv::Vec<double, 2> vector () const;
    /**
     * returns the distance to an other point
     * @return disance
     **/
    double  distanceTo ( const Point2D &p ) const;
    /**
     * returns a cv::Point_<double> reference
     * @return cv
     **/
    const cv::Point_<double>  &cv () const;
    /**
     * returns a cv::Point_<double> reference
     * @return cv
     **/
    cv::Point_<double>  &cv () ;
    /**
     * checks if a point is within a rectangle
     * @param x0 top left x
     * @param y0 top left y
     * @param x1 bottom right x
     * @param y1 bottom right y
     * @return true if inside
     **/
    bool inside ( double x0, double y0, double x1, double y1 ) const;
    /**
     * Stream extraction
     * @param os outputstream
     * @param o object
     * @return stream
     **/
    friend std::ostream &operator << ( std::ostream &os, const Point2D &o ) {
        os << "[" << o.x() <<  ", " << o.y() << "]";
        return os;
    }
};

};

/**
 * overloads the * operator to allow the mutlipication fo the homogeneous point class with an opencv matrix 
 * @param a
 * @param b
 * @return point
 **/
namespace cv {
template<typename _Tp> static inline tuw::Point2D operator * ( const Matx<_Tp, 3, 3>& a, const tuw::Point2D& b ) {
    Matx<_Tp, 3, 1> c ( a, b, Matx_MatMulOp() );
    return reinterpret_cast<const tuw::Point2D&> ( c );
}
}
#endif //POINT2D_H

