#include "tuw_geometry/line2d.h"

using namespace tuw;
Line2D::Line2D() {
};
Line2D::Line2D ( const Line2D &l ) : cv::Vec<double,3> ( l ) {
};
Line2D::Line2D ( cv::Vec<double,3> &l, bool normalize )
    : cv::Vec<double,3> ( l ) {
    if ( normalize ) this->normalize();
};
Line2D::Line2D ( const double &x0, const double &y0, const double &x1, const double &y1, bool normalize ) {
    set ( x0,y0,x1,y1, normalize );
}
Line2D::Line2D ( const Point2D &p0, const Point2D &p1, bool normalize ) {
    set ( p0, p1, normalize );
}
double &Line2D::a() {
    return this->val[0];
}
const double &Line2D::a() const {
    return this->val[0];
}
double &Line2D::b() {
    return this->val[1];
}
const double &Line2D::b() const {
    return this->val[1];
}
double &Line2D::c() {
    return this->val[2];
}
const double &Line2D::c() const {
    return this->val[2];
}
void Line2D::normalize() {
    double r = sqrt ( this->val[0]*this->val[0] + this->val[1]*this->val[1] );
    this->val[0] /= r, this->val[1] /= r, this->val[2] /= r;
}
double Line2D::distanceTo ( const double &x, const double &y )  const {
    return this->val[0]*x + this->val[1]*y + this->val[2];
}
/** @pre normalize */
double Line2D::distanceTo ( const Point2D &p )  const {
    return distanceTo ( p.x(), p.y() );
}
/** @pre pointOnLine */
Point2D Line2D::pointOnLine ( const double &x, const double &y ) const {
    double d = distanceTo ( x,y );
    return Point2D ( x - d * a(), y - d * b() );
}
/** @pre pointOnLine */
Point2D Line2D::pointOnLine ( const Point2D &p ) const {
    return pointOnLine ( p.x(), p.y() );
}
Point2D Line2D::intersection ( const Line2D &l ) const {
    cv::Vec<double,3> h = this->cross ( l );
    return Point2D ( h[0]/h[2],h[1]/h[2] );
}
cv::Vec<double,2> Line2D::normal() const {
    return cv::Vec<double,2> ( this->val[0], this->val[1] );
}
Line2D &Line2D::set ( const double &x0, const double &y0, const double &x1, const double &y1, bool normalize ) {
    this->val[0] = y0-y1, this->val[1] = x1-x0, this->val[2] = x0*y1-y0*x1; /// cross product with homogenios vectors
    if ( normalize ) this->normalize();
    return *this;
}
Line2D &Line2D::set ( const Point2D &p0, const Point2D &p1, bool normalize ) {
    return set ( p0.x(), p0.y(), p1.x(), p1.y(), normalize );
}
cv::Vec<double,3> &Line2D::cv () {
    return *this;
}
const cv::Vec<double,3> &Line2D::cv () const {
    return *this;
}
Polar2D Line2D::toPolar() const{
    cv::Vec<double,2> dv = normal()*(-c());
    double rho = fabs(c());
    double alpha = atan2(dv(1),dv(0));
    return Polar2D(alpha, rho);
}
