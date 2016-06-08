#include "tuw_geometry/pose2d.h"
using namespace tuw;

Pose2D::Pose2D() : position_(), orientation_ ( 0 ) {};
Pose2D::Pose2D ( const Point2D &p, double orientation_ ) : position_ ( p ), orientation_ ( orientation_ ) {};
Pose2D::Pose2D ( const Pose2D &p ) : position_ ( p.position_ ), orientation_ ( p.orientation_ ) {};
Pose2D::Pose2D ( double x, double y, double orientation_ ) : position_ ( x,y ), orientation_ ( orientation_ ) {};
Pose2D::Pose2D ( const cv::Vec<double, 3> &s): position_ ( s(0),s(1) ), orientation_ ( s(2) ) {};

/** set the pose
  * @param x
  * @param y
  * @param phi (orientation_)
  * @return this reference
  **/
Pose2D &Pose2D::set ( const double &x, const double &y, const double &phi ) {
    position_ = cv::Vec<double, 3> ( x, y, 1 ), orientation_ = phi;
    return *this;
}
/**
 * set the pose based on two points in world coordinates
 * @param position
 * @param point_ahead
 * @return this reference
 **/
Pose2D &Pose2D::set ( const Point2D &position, const Point2D &point_ahead ) {
    position_.set ( position.x(), position.y() );
    double dx = point_ahead.x() - position.x(), dy = point_ahead.y() - position.y();
    orientation_ = atan2 ( dy,dx );
    return *this;
}
/** set the pose
  * @param p pose
  * @return this reference
  **/
Pose2D &Pose2D::set ( const Pose2D &p ) {
    position_ = p.position_, orientation_ = p.orientation_;
    return *this;
}
/** location as vector
  * @return translational
  **/
const Point2D &Pose2D::position () const {
    return position_;
}
/** point infront of the pose
 * @param d distance ahead
 * @return point
 **/
Point2D Pose2D::point_ahead ( double d ) const {
    return Point2D ( x() + cos ( theta() ) * d, y() + sin ( theta() ) * d );
}
/** translational x component
  * @return x component
  **/
const double &Pose2D::x () const {
    return position_[0];
}
/** translational y component
  * @return y component
  **/
const double &Pose2D::y () const {
    return position_[1];
}
/** roational component
  * @return rotation
  **/
const double &Pose2D::theta () const {
    return orientation_;
}
/** location as vector
  * @return translational
  **/
Point2D &Pose2D::position () {
    return position_;
}
/** translational x component
  * @return x component
  **/
double &Pose2D::x () {
    return position_[0];
}
/** translational y component
  * @return y component
  **/
double &Pose2D::y () {
    return position_[1];
}
/** roational component
  * @return rotation
  **/
double &Pose2D::theta () {
    return orientation_;
}
/** normalizes the orientation value betwenn -PI and PI
  **/
void Pose2D::normalizeOrientation () {
    angle_normalize ( orientation_, -M_PI, +M_PI );
}
/** computes a transformation matrix for a 2D position
  * @return transformation matrix
  **/
Tf2D Pose2D::tf () const {
    double c = cos ( orientation_ ),  s = sin ( orientation_ );
    return cv::Matx33d ( c, -s, x(), s, c, y(), 0, 0, 1. );
}
/** computes a transformation matrix for a 2D pose
  * @return transformation matrix
  **/
cv::Matx44d Pose2D::tf2 () const {
    double c = cos ( orientation_ ),  s = sin ( orientation_ );
    return cv::Matx44d ( c, -s, 0, x(), s, c, 0, y(), 0, 0, 1, theta(), 0, 0, 0, 1 );
}
/**
 * retuns a state vector [x, y, theta]
 * @return state vector [x, y, theta]
 **/
cv::Vec<double, 3> Pose2D::state_vector () const {
    return cv::Vec<double, 3> ( x(), y(), theta() );
}
/** 
 * retuns the homogeneous vector [x, y, theta, 1] 
 * @return homogeneous vector [x, y, theta, 1] 
 **/
cv::Vec<double, 4> Pose2D::hv () const {
    return cv::Vec<double, 4> ( x(), y(), theta(), 1 );
}
/**
* invert pose
* @return inverted pose
**/
Pose2D Pose2D::inv () const {
    Pose2D p ( -this->x(), -this->y(), -this->theta() );
    return p;
}

/**
 * adds a state vector [x, y, theta]
 * @param s object
 * @return this
 **/
Pose2D &Pose2D::operator += ( const cv::Vec<double, 3> &s ) {
    this->x() += s.val[0], this->y() += s.val[1], this->theta() += s.val[2];
    angle_normalize ( this->theta() );
    return *this;
}
/**
 * substracts a state vector [x, y, theta]
 * @param s object
 * @return this
 **/
Pose2D &Pose2D::operator -= ( const cv::Vec<double, 3> &s ) {
    this->x() -= s.val[0], this->y() -= s.val[1], angle_difference ( this->theta(), s.val[2] );
    return *this;
}
