#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <tuw_geometry/tuw_geometry.h>
#include <tuw_local_planner/LocalPlannerConfig.h>

namespace tuw {
#define LOCAL_PLANNER_WIDTH (0.5)
#define LOCAL_PLANNER_V     (1.0)
#define LOCAL_PLANNER_W     (0.2)

/**
 * Robot class
 */
class LocalPlanner {
public:
    enum ControlMode {
        STOP = 0,
        DEMO = 1,
        WANDERER = 2,
    };
    static std::map<ControlMode, std::string> ControlModeName_;

    LocalPlanner(const std::string &ns); /// Constructor
    void init();                         /// initialization
    void ai();                           /// artificial intelligence calls a behaviour
    void plot();                         /// plots sensor input

protected:

    Command cmd_;  /// output variables  v, w
    unsigned long loop_count_; /// counts the filter cycles

    MeasurementLaser measurement_laser_;        /// laser measurements
    MeasurementFiducial measurement_fiducial_;  /// fiducial measurements

    Figure figure_local_;  /// Figure for data visualization

    void ai_demo();        /// Demo behaviour
    void ai_wanderer();    /// Wanderer behaviour
    void plotLocal();      /// plots sensor input in robot coordinates
    tuw_local_planner::LocalPlannerConfig config_;

private:
    enum WandererState {
        INIT = 0,
        GO = 1,
        TURN = 2,
    };
    static std::map<WandererState, std::string> WandererStateName_;

    static std::random_device rd_;  /// random number device
    static std::mt19937 generator_; /// random number generator

    WandererState wanderer_state_;
};
}

#endif // PLANNER_LOCAL_H

