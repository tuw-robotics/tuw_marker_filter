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
    sub_marker_ = n.subscribe ( "marker", 5, &LocalPlannerNode::callbackMarker, this );

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
        measurement_laser_.pose2d() = Pose2D ( transform.getOrigin().getX(),
                                             transform.getOrigin().getY(),
                                             transform.getRotation().getAngle() );
    } catch ( tf::TransformException &ex ) {
        ROS_ERROR ( "%s",ex.what() );
        measurement_laser_.pose2d() = Pose2D ( 0.225,  0, 0 );
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
 * copies incoming marker messages to the base class
 * @param _marker
 **/
void LocalPlannerNode::callbackMarker ( const marker_msgs::MarkerDetection &_marker ) {
    try {
        tf::StampedTransform transform;
        tf_listener_->lookupTransform ( tf::resolve(n_.getNamespace(),"base_link"), _marker.header.frame_id, ros::Time ( 0 ), transform );
        measurement_marker_.pose2d() = Pose2D ( transform.getOrigin().getX(),
                                                transform.getOrigin().getY(),
                                                tf::getYaw ( transform.getRotation() ) );
    } catch ( tf::TransformException &ex ) {
        ROS_ERROR ( "%s",ex.what() );
        measurement_marker_.pose2d() = Pose2D ( 0.225, 0, 0 );
    }

    measurement_marker_.angle_min() = -_marker.fov_horizontal/2.;
    measurement_marker_.angle_max() = +_marker.fov_horizontal/2.;
    measurement_marker_.range_min() = _marker.distance_min;
    measurement_marker_.range_max() = _marker.distance_max;
    measurement_marker_.range_max_id() = _marker.distance_max_id;
    measurement_marker_.stamp() = _marker.header.stamp.toBoost();
    measurement_marker_.resize ( _marker.markers.size() );

    for ( int i = 0; i < measurement_marker_.size(); i++ ) {
        tf::Vector3 v;
        tf::pointMsgToTF ( _marker.markers[i].pose.position, v );
        double orientation = tf::getYaw ( _marker.markers[i].pose.orientation );

        measurement_marker_[i].id = _marker.markers[i].ids[0];
        measurement_marker_[i].length = v.length();
        measurement_marker_[i].angle = atan2 ( v.getY(), v.getX() );
        measurement_marker_[i].orientation = orientation;
        measurement_marker_[i].pose = Pose2D ( v.getX(), v.getY(), orientation );
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
