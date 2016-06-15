#include "tuw_marker_slam/tuw_marker_slam.h"

using namespace tuw;

SLAM::SLAM()
    : loop_count_ ( 0 ) {
}

void SLAM::cycle() {
    slam_technique_->cycle ( yt_, C_Yt_, ut_, zt_ );
    loop_count_++;
}
