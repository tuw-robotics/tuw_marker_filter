#include "tuw_local_planner/tuw_local_planner.h"
#include <opencv/cv.h>
#include <boost/concept_check.hpp>

using namespace cv;
using namespace tuw;

std::random_device LocalPlanner::rd_;
std::mt19937 LocalPlanner::generator_ ( rd_() );

std::map<LocalPlanner::ControlMode, std::string> LocalPlanner::ControlModeName_ = {
    {DEMO, "DEMO"},
    {STOP, "STOP"},
    {WANDERER, "WANDERER"}
};

std::map<LocalPlanner::WandererState, std::string> LocalPlanner::WandererStateName_ = {
    {INIT, "INIT"},
    {GO, "GO"},
    {TURN, "TURN"}
};

LocalPlanner::LocalPlanner ( const std::string &ns )
    : loop_count_ ( 0 )
    , figure_local_ ( ns + ", Local View" ) {

}

void LocalPlanner::init() {

    figure_local_.init ( config_.map_pix_x, config_.map_pix_y,
                         config_.map_min_x, config_.map_max_x,
                         config_.map_min_y, config_.map_max_y,
                         config_.map_rotation + M_PI/2.0,
                         config_.map_grid_x, config_.map_grid_y );

    cv::putText ( figure_local_.background(), ControlModeName_[ ( ControlMode ) config_.mode],
                  cv::Point ( 5,figure_local_.view().rows-10 ),
                  cv::FONT_HERSHEY_PLAIN, 1, Figure::black, 1, CV_AA );

    wanderer_state_ = INIT;
}

void LocalPlanner::plot() {
    if ( config_.plot_data ) plotLocal();
    cv::waitKey ( 10 );
}

void LocalPlanner::plotLocal() {
    figure_local_.clear();

    /// plot end points of laser measurement beams
    for ( size_t i = 0; i < measurement_laser_.size(); i++ ) {
        Point2D p = measurement_laser_.getSensorPose().tf() * measurement_laser_[i].end_point;
        figure_local_.circle ( p, 1, Figure::red );
    }

    /// plot found fiducials
    for ( size_t i = 0; i < measurement_fiducial_.size(); i++ ) {
        Point2D p = measurement_fiducial_.getSensorPose().tf() * measurement_fiducial_[i].pose.position();
        figure_local_.circle ( p, 3, Figure::blue );
    }

    cv::imshow ( figure_local_.title(),figure_local_.view() );
}

void LocalPlanner::ai() {
    if ( measurement_laser_.empty() ) {
        cmd_.set ( 0, 0 );
        return;
    }
    switch ( config_.mode ) {
    case STOP:
        cmd_.set ( 0, 0 );
        break;
    case DEMO:
        ai_demo();
        break;
    case WANDERER:
        ai_wanderer();
        break;
    default:
        cmd_.set ( 0, 0 );
    }
    loop_count_++;
}

/**
* Demo
**/
void LocalPlanner::ai_demo() {
    double v = 0.0, w = 0.0;
    if ( measurement_laser_.empty() ) {
        v = 0.2, w = -0.02;
    } else {
        if ( measurement_laser_[measurement_laser_.size() / 4].length < 1.0 ) {
            w = 0.4;
        } else {
            v = 0.4;
        }
    }
    cmd_.set ( v, w );
}

/**
* @Wanderer
* writes one or two wanderer behaviours to keep the robot at least 120sec driving without a crash by exploring as much as possible.
* I know it sounds weird but don't get too fancy.
**/
void LocalPlanner::ai_wanderer() {
    static const double theta = asin ( LOCAL_PLANNER_WIDTH/ ( 2*config_.safety_distance ) );
    static ros::Time timestamp_turn;
    static double alpha;
    bool obstacle;
    double l, r;
    std::uniform_real_distribution<double> uniform ( 0, M_PI );

    assert ( config_.safety_distance >= measurement_laser_.range_min() );
    assert ( config_.safety_distance <= measurement_laser_.range_max() );

    // check if there are any obstacles ahead
    obstacle = false;
    for ( int i = 0; i < measurement_laser_.size(); i++ ) {
        if ( measurement_laser_[i].angle < 0 )
            r += measurement_laser_[i].length;
        else
            l += measurement_laser_[i].length;

        if ( -theta <= measurement_laser_[i].angle && measurement_laser_[i].angle <= theta ) {
            if ( !obstacle && measurement_laser_[i].length <= config_.safety_distance )
                obstacle = true;
        }
    }

    // Simple state machine
    switch ( wanderer_state_ ) {
    case INIT:
        if ( obstacle ) {
            if ( l < r )
                cmd_.set ( 0, -LOCAL_PLANNER_W );
            else
                cmd_.set ( 0,  LOCAL_PLANNER_W );

            timestamp_turn = ros::Time::now();
            alpha = uniform ( generator_ );
            wanderer_state_ = TURN;
        } else {
            cmd_.set ( LOCAL_PLANNER_V, -1.0 * ( rand() % 2 ) * 0.2 * ( rand() % 2 ) );
            //cmd_.set ( LOCAL_PLANNER_V, 0 );
            //cmd_.set ( LOCAL_PLANNER_V, LOCAL_PLANNER_W );
            wanderer_state_ = GO;
        }
        break;
    case GO:
        if ( obstacle ) {
            if ( l < r )
                cmd_.set ( 0, -LOCAL_PLANNER_W );
            else
                cmd_.set ( 0,  LOCAL_PLANNER_W );

            timestamp_turn = ros::Time::now();
            alpha = uniform ( generator_ );
            wanderer_state_ = TURN;
        } else if ( loop_count_ % 50 == 0 ) {
            cmd_.set ( LOCAL_PLANNER_V, -1.0 * ( rand() % 2 ) * 0.2 * ( rand() % 2 ) );
            //cmd_.set ( LOCAL_PLANNER_V, 0 );
            //cmd_.set ( LOCAL_PLANNER_V, LOCAL_PLANNER_W );
        }
        break;
    case TURN:
        if ( !obstacle && (ros::Time::now() - timestamp_turn).toSec() * LOCAL_PLANNER_W > alpha ) {
            cmd_.set ( LOCAL_PLANNER_V, -1.0 * ( rand() % 2 ) * 0.2 * ( rand() % 2 ) );
            //cmd_.set ( LOCAL_PLANNER_V, 0 );
            //cmd_.set ( LOCAL_PLANNER_V, LOCAL_PLANNER_W );
            wanderer_state_ = GO;
        }
        break;
    default:
        assert ( false );
    }
}
