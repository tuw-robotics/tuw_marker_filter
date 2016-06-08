#ifndef POSE2D_H
#define POSE2D_H

#include <memory>
#include <opencv2/core/core.hpp>
#include <tuw_geometry/point2d.h>
namespace tuw {
class Pose2D;
using Pose2DPtr = std::shared_ptr< Pose2D >;
using Pose2DConstPtr = std::shared_ptr< Pose2D const>;


/**
 * class to represent a pose in 2D space
 **/
class Pose2D {
protected:
    Point2D position_;    /// position
    double orientation_;  /// rotation in rad
public:
    Pose2D();
    Pose2D ( const Point2D &p, double orientation );
    Pose2D ( const Pose2D &p );
    Pose2D ( double x, double y, double orientation );
    Pose2D ( const cv::Vec<double, 3> &s);
    /** 
     * set the pose
     * @param x
     * @param y
     * @param phi (orientation)
     * @return this reference
     **/
    Pose2D &set ( const double &x, const double &y, const double &phi ) ;
    /** 
     * set the pose based on two points in world coordinates
     * @param position 
     * @param point_ahead
     * @return this reference
     **/
    Pose2D &set ( const Point2D &position, const Point2D &point_ahead) ;
    
    /** 
     * set the pose
     * @param p pose
     * @return this reference
     **/
    Pose2D &set ( const Pose2D &p ) ;
    
    /** 
     * position
     * @return translational
     **/
    const Point2D &position () const;
    
    /** 
     * location as vector
     * @return translational
     **/
    Point2D &position ();
    
    /** 
     * point in front of the pose
     * @param d distance ahead
     * @return point
     **/
    const double &x () const;
    
    /** 
     * translational y component
     * @return y component
     **/
    double &x ();
    
    /** 
     * translational y component
     * @return y component
     **/
    const double &y () const;
    
    /** 
     * rotational component
     * @return rotation
     **/
    double &y ();
    
    /** 
     * rotational component
     * @return rotation
     **/
    const double &theta () const;
    
    /** 
     * position
     * @return translational
     **/
    double &theta ();
    
    /** 
     * computes a transformation matrix
     * @return transformation
     **/
    Point2D point_ahead ( double d = 1. ) const;
    
    /** 
     * normalizes the orientation value betwenn -PI and PI
      **/
    void normalizeOrientation ();
    
    /** 
     * computes a transformation matrix for a 2D position
     * @return transformation matrix
     **/
    Tf2D tf () const;
    
    /** 
     * computes a transformation matrix for a 2D pose
     * @return transformation matrix
     **/
    cv::Matx44d tf2 () const;
    
    /** 
     * retuns a state vector [x, y, theta] 
     * @return state vector [x, y, theta] 
     **/
    cv::Vec<double, 3> state_vector () const;

    /** 
     * retuns the homogeneous vector [x, y, theta, 1] 
     * @return homogeneous vector [x, y, theta, 1] 
     **/
    cv::Vec<double, 4> hv () const;

    /** 
     * invert pose
     * @return inverted pose
     **/
    Pose2D inv () const;

    
    /** 
     * adds a state vector [x, y, theta]
     * the orientation will be normalized between -PI and PI
     * @param s object
     * @return thisl
     **/
    Pose2D &operator += ( const cv::Vec<double, 3> &s);
    
    /**
    * substracts a state vector [x, y, theta] 
    * the orientation will be normalized between -PI and PI
    * @param s object
    * @return this
    **/
    Pose2D &operator -= ( const cv::Vec<double, 3> &s ) ;
    /** 
     * Stream extraction
     * @param os outputstream
     * @param o object
     * @return stream
     **/
    friend std::ostream &operator << ( std::ostream &os, const Pose2D &o ) {
        os << "[" << o.x() <<  ", " << o.y() <<  ", " << o.theta()  << "]";
        return os;
    }
};

}
#endif //POSE2D_H

