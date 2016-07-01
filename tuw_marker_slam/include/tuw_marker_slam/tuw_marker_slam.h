#ifndef TUW_MARKER_SLAM_H
#define TUW_MARKER_SLAM_H

#include <tuw_geometry/tuw_geometry.h>
#include <opencv2/core/core.hpp>

#include "tuw_marker_slam/slam_technique.h"
#include "tuw_marker_slam/SLAMConfig.h"

namespace tuw {
/**
 * Class for SLAM (independent of used technique)
 */
class SLAM {
public:
    SLAM ();                                                /// constructor

protected:
    std::vector<Pose2D> yt_;                                /// combined state yt = (xt, mt_1, ..., mt_n) with xt = (x, y, alpha), mt_i = (x_i, y_i, alpha_i)
    cv::Mat_<double> C_Yt_;                                 /// combined covariance matrix of combined state
    Command ut_;                                            /// motion commands v, w at time t
    MeasurementPtr zt_;                                     /// measurements at time t

    unsigned long loop_count_;                              /// counter for the triggered cycles
    SLAMTechniquePtr slam_technique_;                       /// technique used to estimate landmark map and the vehicles pose in it

    tuw_marker_slam::SLAMConfig config_;                    /// global parameters

    void cycle ();                                          /// triggers the SLAM cycle
};
};

#endif // TUW_MARKER_SLAM_H
