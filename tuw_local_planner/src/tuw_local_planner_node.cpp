#include "tuw_local_planner_node.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <tf/transform_datatypes.h>

using namespace tuw;
int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "planner_local" );  /// initializes the ros node with default name
    ros::NodeHandle n;
    LocalPlannerNode planner ( n );
    planner.init();
    ros::Rate rate ( 10 );  /// ros loop frequence synchronized with the wall time (simulated time)

    while ( ros::ok() ) {

        /// calls your loop
        planner.ai();

        /// sets and publishes velocity commands
        planner.publishMotion();

        /// plots measurements
        planner.plot();

        /// calls all callbacks waiting in the queue
        ros::spinOnce();

        /// sleeps for the time remaining to let us hit our publish rate
        rate.sleep();
    }

    return 0;
}

/**
 * Constructor
 **/
LocalPlannerNode::LocalPlannerNode ( ros::NodeHandle & n )
    : LocalPlanner ( ros::NodeHandle ( "~" ).getNamespace() ),
      n_ ( n ),
      n_param_ ( "~" ) {

    /// subscribe to sensor data
    sub_laser_ = n.subscribe ( "laser", 5, &LocalPlannerNode::callbackLaser, this );
    sub_fiducial_ = n.subscribe ( "fiducial", 5, &LocalPlannerNode::callbackFiducial, this );

    /// defines a publisher for velocity commands
    pub_cmd_ = n.advertise<geometry_msgs::Twist> ( "cmd_vel", 1 );

    /// subscribe to transforamtions
    tf_listener_ = std::make_shared<tf::TransformListener>();

    reconfigureFnc_ = boost::bind ( &LocalPlannerNode::callbackConfigLocalPlanner, this,  _1, _2 );
    reconfigureServer_.setCallback ( reconfigureFnc_ );
}

void LocalPlannerNode::callbackConfigLocalPlanner ( tuw_local_planner::LocalPlannerConfig &config, uint32_t level ) {
    ROS_INFO ( "callbackConfigLocalPlanner!" );
    config_ = config;
    init();
}

/**
 * copies incoming laser messages to the base class
 * @param laser
 **/
void LocalPlannerNode::callbackLaser ( const sensor_msgs::LaserScan &_laser ) {
    double alpha, r;

    try {
        tf::StampedTransform transform;
        tf_listener_->lookupTransform ( tf::resolve(n_.getNamespace(),"base_link"), _laser.header.frame_id, ros::Time ( 0 ), transform );
        measurement_laser_.getSensorPose() = Pose2D ( transform.getOrigin().getX(),
                                             transform.getOrigin().getY(),
                                             transform.getRotation().getAngle() );
    } catch ( tf::TransformException &ex ) {
        ROS_ERROR ( "%s",ex.what() );
        measurement_laser_.getSensorPose() = Pose2D ( 0.225,  0, 0 );
    }

    measurement_laser_.range_max() = _laser.range_max;
    measurement_laser_.range_min() = _laser.range_min;
    measurement_laser_.resize ( _laser.ranges.size() );
    alpha = _laser.angle_min;
    for ( int i = 0; i < measurement_laser_.size(); i++ ) {
        r = _laser.ranges[i];

        measurement_laser_[i].length = r;
        measurement_laser_[i].angle = alpha;
        measurement_laser_[i].end_point = Point2D ( r*cos ( alpha ), r*sin ( alpha ) );

        alpha += _laser.angle_increment;
    }
}

/**
 * copies incoming fiducial messages to the base class
 * @param laser
 **/
void LocalPlannerNode::callbackFiducial ( const sensor_msgs::FiducialDetection &_fiducial ) {
    try {
        tf::StampedTransform transform;
        tf_listener_->lookupTransform ( tf::resolve(n_.getNamespace(),"base_link"), _fiducial.header.frame_id, ros::Time ( 0 ), transform );
        measurement_fiducial_.getSensorPose() = Pose2D ( transform.getOrigin().getX(),
                                                transform.getOrigin().getY(),
                                                tf::getYaw ( transform.getRotation() ) );
    } catch ( tf::TransformException &ex ) {
        ROS_ERROR ( "%s",ex.what() );
        measurement_fiducial_.getSensorPose() = Pose2D ( 0.225, 0, 0 );
    }

    measurement_fiducial_.angle_min() = _fiducial.angle_min;
    measurement_fiducial_.angle_max() = _fiducial.angle_max;
    measurement_fiducial_.range_min() = _fiducial.range_min;
    measurement_fiducial_.range_max() = _fiducial.range_max;
    measurement_fiducial_.range_max_id() = _fiducial.range_max_id;
    measurement_fiducial_.sigma_radial() = _fiducial.sigma_radial;
    measurement_fiducial_.sigma_polar() = _fiducial.sigma_polar;
    measurement_fiducial_.sigma_azimuthal() = _fiducial.sigma_azimuthal;
    measurement_fiducial_.sigma_roll() = _fiducial.sigma_roll;
    measurement_fiducial_.sigma_pitch() = _fiducial.sigma_pitch;
    measurement_fiducial_.sigma_yaw() = _fiducial.sigma_yaw;
    measurement_fiducial_.stamp() = _fiducial.header.stamp.toBoost();
    measurement_fiducial_.resize ( _fiducial.fiducials.size() );

    for ( int i = 0; i < measurement_fiducial_.size(); i++ ) {
        tf::Vector3 v;
        tf::pointMsgToTF ( _fiducial.fiducials[i].pose.position, v );
        double orientation = tf::getYaw ( _fiducial.fiducials[i].pose.orientation );

        measurement_fiducial_[i].id = _fiducial.fiducials[i].id;
        measurement_fiducial_[i].length = v.length();
        measurement_fiducial_[i].angle = atan2 ( v.getY(), v.getX() );
        measurement_fiducial_[i].orientation = orientation;
        measurement_fiducial_[i].pose = Pose2D ( v.getX(), v.getY(), orientation );
    }
}


/**
 * Publishes motion commands for a robot
 **/
void LocalPlannerNode::publishMotion () {
    geometry_msgs::Twist cmd;

    /// creates motion command
    cmd.linear.x = cmd_.v();
    cmd.linear.y = 0.;
    cmd.angular.z = cmd_.w();

    /// publishes motion command
    pub_cmd_.publish ( cmd );
}
