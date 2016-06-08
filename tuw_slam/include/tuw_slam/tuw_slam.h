#ifndef TUW_SLAM_H
#define TUW_SLAM_H

#include <tuw_geometry/tuw_geometry.h>
#include <opencv2/core/core.hpp>
#include "tuw_slam/slam_technique.h"
#include "tuw_slam/SLAMConfig.h"

namespace tuw {
/**
 * Class for SLAM independent to the used technique
 */
class SLAM {
public:
    SLAM ();                                                /// constructor

protected:
    std::vector<Pose2D> yt_;                                /// combined state yt = (xt, mt_1, ..., mt_n) with xt = (x, y, alpha), mt_i = (x_i, y_i, alpha_i)
    cv::Mat_<double> C_Yt_;                                 /// combined covariance matrix
    Command ut_;                                            /// motion commands v, w
    MeasurementPtr zt_;                                     /// observations

    unsigned long loop_count_;                              /// counts the filter cycles
    SLAMTechniquePtr slam_technique_;                       /// filter used to estimate the vehicles pose

    tuw_slam::SLAMConfig config_;                           /// global parameters

    /**
     * starts the localization and mapping
     */
    void cycle ();
};
};

#endif // TUW_SLAM_H
