#include "tuw_marker_slam/slam_technique.h"

using namespace tuw;

std::map<SLAMTechnique::Type, std::string> SLAMTechnique::TypeName_ = {
    {EKF, "EKF"},
};

SLAMTechnique::SLAMTechnique ( Type type ) :
    reset_ ( true ),
    type_ ( type ),
    timestamp_last_update_() {
};

void SLAMTechnique::reset ( ) {
    reset_ = true;
}

SLAMTechnique::Type  SLAMTechnique::getType() const {
    return type_;
}

const std::string SLAMTechnique::getTypeName() const {
    return TypeName_[ type_ ];
}

const boost::posix_time::ptime& SLAMTechnique::time_last_update() const {
    return timestamp_last_update_;
}

bool SLAMTechnique::updateTimestamp ( const boost::posix_time::ptime& t )  {
    if ( timestamp_last_update_.is_not_a_date_time() ) timestamp_last_update_ = t;
    if ( timestamp_last_update_ < t ) {
        duration_last_update_ = t - timestamp_last_update_;
        timestamp_last_update_ = t;
        return true;
    } else {
        return false;
    }
}
