#include "tuw_geometry/linesegment2d.h"

using namespace tuw;
LineSegment2D::LineSegment2D() {};
LineSegment2D::LineSegment2D ( const LineSegment2D &l )
    : Line2D ( l.line() )
    , p0_ ( l.p0() )
    , p1_ ( l.p1() ) {
};
LineSegment2D::LineSegment2D ( const Point2D &p0, const Point2D &p1 )
    : Line2D ( p0, p1, true )
    , p0_ ( p0 )
    , p1_ ( p1 ) {
};
LineSegment2D::LineSegment2D ( const double &x0, const double &y0, const double &x1, const double &y1 )
    : Line2D ( x0, y0, x1, y1, true )
    , p0_ ( x0, y0 )
    , p1_ ( x1, y1 ) {
};
const double &LineSegment2D::x0() const {
    return  p0_.x();
}
const double &LineSegment2D::y0() const {
    return  p0_.y();
}
const double &LineSegment2D::x1() const {
    return p1_.x();
}
const double &LineSegment2D::y1() const {
    return p1_.y();
}
double LineSegment2D::angle() const {
    double dx = p1_.x() - p0_.x();
    double dy = p1_.y() - p0_.y();
    return atan2 ( dy,dx );
}
Point2D LineSegment2D::pc() const {
    double dx = p1_.x() - p0_.x();
    double dy = p1_.y() - p0_.y();
    return Point2D ( p0_.x() + dx/2., p0_.y() + dy/2. );
}
const Point2D &LineSegment2D::p0() const {
    return p0_;
}
const Point2D &LineSegment2D::p1() const {
    return p1_;
}
const Line2D &LineSegment2D::line() const {
    return *this;
}
const double LineSegment2D::length() const {
    return p0_.distanceTo ( p1_ );
}
/// comparison operator @return true on equal
bool LineSegment2D::operator == ( const LineSegment2D& o ) const {
    return p0() == o.p0() && p1() == o.p1();
}
LineSegment2D& LineSegment2D::set ( const double &x0, const double &y0, const double &x1, const double &y1 ) {
    Line2D::set ( x0, y0, x1, y1, true );
    p0_.set ( x0, y0 ), p1_.set ( x1,y1 );
    return *this;
}
LineSegment2D& LineSegment2D::set ( const Point2D &p0, const Point2D &p1 ) {
    set ( p0.x(), p0.y(), p1.x(), p1.y() );
    return *this;
}
/** computes distance to line segment
 * @see http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
 * @param p point
 * @param dx vector to point x
 * @param dx vector to point y
 * @return distance to line between the segment endpoints or the distance to the nearest endpoints **/
double LineSegment2D::distanceTo ( const Point2D &p, double &dx, double &dy ) const {
    double px = x1()-x0();
    double py = y1()-y0();
    double l2 = px*px + py*py;
    double u = ( ( p.x() - x0() ) * px + ( p.y() - y0() ) * py ) / l2;
    if ( u > 1 ) u = 1;
    else if ( u < 0 ) u = 0;
    double xk = x0() + u * px;
    double yk = y0() + u * py;
    dx = xk - p.x();
    dy = yk - p.y();
    return sqrt ( dx*dx + dy*dy );
}
/** computes distance to line segment
 * @see http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
 * @param p point
 * @return distance to line between the segment endpoints or the distance to the nearest endpoints **/
double LineSegment2D::distanceTo ( const Point2D &p ) const {
    double dx, dy;
    distanceTo ( p, dx, dy );
}

