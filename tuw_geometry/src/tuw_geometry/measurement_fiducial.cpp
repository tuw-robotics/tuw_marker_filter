#include <tuw_geometry/measurement_fiducial.h>
using namespace tuw;

MeasurementFiducial::MeasurementFiducial()
    :Measurement ( Type::FIDUCIAL ) {
}
MeasurementFiducial::MeasurementFiducial ( const MeasurementFiducial& o )
    :Measurement ( o ), range_max_id_ ( o.range_max_id_ ), range_max_ ( o.range_max_ ), range_min_ ( o.range_min_ ), fiducials_ ( o.fiducials_ ) {
}
void MeasurementFiducial::resize ( size_t n ) {
    fiducials_.resize ( n );
}
double &MeasurementFiducial::angle_min() {
    return angle_min_;
}
double &MeasurementFiducial::angle_max() {
    return angle_max_;
}
double &MeasurementFiducial::range_min() {
    return range_min_;
}
const double &MeasurementFiducial::range_min() const {
    return range_min_;
}
double &MeasurementFiducial::range_max() {
    return range_max_;
}
const double &MeasurementFiducial::range_max() const {
    return range_max_;
}
double &MeasurementFiducial::range_max_id() {
    return range_max_id_;
}
const double &MeasurementFiducial::range_max_id() const {
    return range_max_id_;
}
double &MeasurementFiducial::sigma_radial() {
    return sigma_radial_;
}
const double &MeasurementFiducial::sigma_radial() const {
    return sigma_radial_;
}
double &MeasurementFiducial::sigma_polar() {
    return sigma_polar_;
}
const double &MeasurementFiducial::sigma_polar() const {
    return sigma_polar_;
}
double &MeasurementFiducial::sigma_azimuthal() {
    return sigma_azimuthal_;
}
const double &MeasurementFiducial::sigma_azimuthal() const {
    return sigma_azimuthal_;
}
double &MeasurementFiducial::sigma_roll() {
    return sigma_roll_;
}
const double &MeasurementFiducial::sigma_roll() const {
    return sigma_roll_;
}
double &MeasurementFiducial::sigma_pitch() {
    return sigma_pitch_;
}
const double &MeasurementFiducial::sigma_pitch() const {
    return sigma_pitch_;
}
double &MeasurementFiducial::sigma_yaw() {
    return sigma_yaw_;
}
const double &MeasurementFiducial::sigma_yaw() const {
    return sigma_yaw_;
}
bool MeasurementFiducial::empty() const {
    return fiducials_.empty();
}
size_t MeasurementFiducial::size() const {
    return fiducials_.size();
}
/** Array operation
 * @param i entry to return
 * @return reference to the element
 **/
MeasurementFiducial::Fiducial& MeasurementFiducial::operator[] ( int i ) {
    return fiducials_[i];
}
/** Array operation const version
 * @param i entry to return
 * @return reference to the element
 **/
const MeasurementFiducial::Fiducial& MeasurementFiducial::operator[] ( int i ) const {
    return fiducials_[i];
}

