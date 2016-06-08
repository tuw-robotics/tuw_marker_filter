#ifndef POLAR2D_H
#define POLAR2D_H

#include <memory>
#include <tuw_geometry/point2d.h>

namespace tuw {
class Polar2D;  /// Prototype
using Polar2DPtr = std::shared_ptr< Polar2D >;
using Polar2DConstPtr = std::shared_ptr< Polar2D const>;

/**
 * class to represent a point with rho and alpha
 **/
class Polar2D : public Point2D {
public:
    Polar2D ();
    Polar2D ( const Point2D &p );
    Polar2D ( double alpha, double rho );
    Polar2D ( double alpha, double rho, double h );

    /**
     * @return alpha
     **/
    const double &alpha () const;
    /**
     * @return alpha
     **/
    double &alpha ();
    /**
     * @return rho component
     **/
    const double &rho () const;
    /**
     * @return rho component
     **/
    double &rho () ;

    /**
     * normalizes the polar system to a positive rho value
     * @return ref to *this
     **/
    Polar2D &nomalize ();
    

private:
    using Point2D::x;
    using Point2D::y;
};

}
#endif //POLAR2D_H

