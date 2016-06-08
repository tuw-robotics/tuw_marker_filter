#include <tuw_geometry/measurement_laser.h>
using namespace tuw;

MeasurementLaser::MeasurementLaser()
    :Measurement ( Type::LASER ) {
}
MeasurementLaser::MeasurementLaser ( const MeasurementLaser& o )
    :Measurement ( o ), range_max_ ( o.range_max_ ), range_min_ ( o.range_min_ ), beam_ ( o.beam_ ) {
}
void MeasurementLaser::resize ( size_t n ) {
    beam_.resize ( n );
}
double &MeasurementLaser::range_max() {
    return range_max_;
}
const double &MeasurementLaser::range_max() const {
    return range_max_;
}
double &MeasurementLaser::range_min() {
    return range_min_;
}
const double &MeasurementLaser::range_min() const {
    return range_min_;
}
bool MeasurementLaser::empty() const {
    return beam_.empty();
}
size_t MeasurementLaser::size() const {
    return beam_.size();
}
/** Array operation
 * @param i entry to return
 * @return reference to the element
 **/
MeasurementLaser::Beam& MeasurementLaser::operator[] ( int i ) {
    return beam_[i];
}
/** Array operation const version
 * @param i entry to return
 * @return reference to the element
 **/
const MeasurementLaser::Beam& MeasurementLaser::operator[] ( int i ) const {
    return beam_[i];
}

