#include <tf/transform_datatypes.h>

#include "tuw_marker_slam_node.h"
#include "tuw_marker_slam/ekf_slam.h"

using namespace tuw;

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "slam" );
    ros::NodeHandle n;
    SLAMNode slam ( n );
    ros::Rate rate ( 10 );

    while ( ros::ok() ) {
        /// localization and mapping
        slam.cycle();

        /// publishes the estimated pose and mapping
        slam.publish();

        /// calls all callbacks waiting in the queue
        ros::spinOnce();

        /// sleep for the time remaining to let us hit our publish rate
        rate.sleep();
    }
    return 0;
}

/**
 * Constructor
 **/
SLAMNode::SLAMNode ( ros::NodeHandle & n )
    : SLAM (),
      n_ ( n ),
      n_param_ ( "~" ) {
    // read in parameters
    int mode;
    n_param_.getParam ( "mode", mode );

    switch ( mode ) {
    case SLAMTechnique::EKF:
        // read in EKF specific parameters
        double beta_1, beta_2, beta_3, beta_4, beta_5, beta_6;
        n_param_.getParam ( "beta_1", beta_1 );
        n_param_.getParam ( "beta_2", beta_2 );
        n_param_.getParam ( "beta_3", beta_3 );
        n_param_.getParam ( "beta_4", beta_4 );
        n_param_.getParam ( "beta_5", beta_5 );
        n_param_.getParam ( "beta_6", beta_6 );

        zt_ = std::make_shared<tuw::MeasurementMarker>();
        slam_technique_ = std::make_shared<tuw::EKFSLAM>( beta_1, beta_2, beta_3, beta_4, beta_5, beta_6 );
        break;
    default:
        ROS_ERROR ( "[%s] mode %i is not supported", ros::this_node::getName().c_str(), mode );
        return;
    }
    ROS_INFO ( "[%s] mode: %s (%i)", ros::this_node::getName().c_str(), slam_technique_->getTypeName().c_str(), ( int ) slam_technique_->getType() );

    // initialize global variables
    //pose_ground_truth_ = Pose2D ( -3.0, -3.0, M_PI/4 );
    //pose_ground_truth_ = Pose2D ( 0, -2, 0 );
    pose_ground_truth_ = Pose2D ( 0, -5, 0 );
    //pose_ground_truth_ = Pose2D ( 0, -8, 0 );
    origin_ = pose_ground_truth_;

    /// subscribes to transforamtions
    tf_listener_ = std::make_shared<tf::TransformListener>();

    /// subscribes to command values
    sub_cmd_ = n.subscribe ( "cmd", 1, &SLAMNode::callbackCmd, this );

    /// subscribes to ground truth data
    sub_ground_truth_ = n.subscribe ( "base_pose_ground_truth", 1, &SLAMNode::callbackGroundTruth, this );

    /// defines publishers for the resulting robot pose
    pub_xt_ = n_param_.advertise<marker_msgs::MarkerWithCovarianceStamped> ( "xt", 1 );
    n_param_.param<std::string> ( "frame_id_map", xt_.header.frame_id, "map" );
    xt_.header.seq = 0;

    /// defines publishers for the resulting landmark poses
    pub_mt_ = n_param_.advertise<marker_msgs::MarkerWithCovarianceArray> ( "mt", 1 );
    n_param_.param<std::string> ( "frame_id_map", mt_.header.frame_id, "map" );
    mt_.header.seq = 0;

    /// start parameter server
    reconfigureFncSLAM_ = boost::bind ( &SLAMNode::callbackConfigSLAM, this,  _1, _2 );
    reconfigureServerSLAM_.setCallback ( reconfigureFncSLAM_ );

    switch ( slam_technique_->getType() ) {
    case SLAMTechnique::EKF:
        /// subscribes to marker detector
        sub_marker_ = n.subscribe ( "marker", 1, &SLAMNode::callbackMarker, this );

        /// start parameter server
        reconfigureServerEKFSLAM_ = std::make_shared< dynamic_reconfigure::Server<tuw_marker_slam::EKFSLAMConfig> > ( ros::NodeHandle ( "~/" + slam_technique_->getTypeName() ) );
        reconfigureFncEKFSLAM_ = boost::bind ( &SLAMNode::callbackConfigEKFSLAM, this,  _1, _2 );
        reconfigureServerEKFSLAM_->setCallback ( reconfigureFncEKFSLAM_ );
        break;
    default:
        assert ( 0 );
    }
}

void SLAMNode::cycle() {
    if ( config_.reset ) {
        origin_ = pose_ground_truth_;
        slam_technique_->reset();
    }

    SLAM::cycle ();
}

/**
 * Publishes the estimated pose
 **/
void SLAMNode::publish () {
    if ( slam_technique_->time_last_update().is_not_a_date_time() ) return;

    // transformations
    tf::Quaternion q;
    tf::Transform trans;

    // publish world -> map transformation
    std::string map;
    n_param_.param<std::string> ( "frame_id_map", map, "map" );
    q.setRPY ( 0.0, 0.0, origin_.theta() );
    trans =  tf::Transform ( q, tf::Point ( origin_.x(), origin_.y(), 0 ) );
    tf_broadcaster_.sendTransform ( tf::StampedTransform ( trans, ros::Time::fromBoost ( slam_technique_->time_last_update() ), "odom", map ) );

    // publish map -> estimated pose transforamtion
    std::string slam_pose;
    n_param_.param<std::string> ( "frame_id_slam_pose", slam_pose, "slam_pose" );
    q.setRPY ( 0.0, 0.0, yt_[0].theta() );
    trans =  tf::Transform ( q, tf::Point (yt_[0].x(), yt_[0].y(), 0 ) );
    tf_broadcaster_.sendTransform ( tf::StampedTransform ( trans, ros::Time::fromBoost ( slam_technique_->time_last_update() ), map, slam_pose ) );

    assert ( yt_.size() > 0 && C_Yt_.rows == 3*yt_.size() && C_Yt_.cols == 3*yt_.size() );

    // publish estimated robot pose and its variance
    xt_.header.stamp = ros::Time::fromBoost ( slam_technique_->time_last_update() );
    xt_.header.seq++;

    xt_.marker.marker.pose.position.x = yt_[0].x();
    xt_.marker.marker.pose.position.y = yt_[0].y();
    xt_.marker.marker.pose.position.z = 0;
    xt_.marker.marker.pose.orientation = tf::createQuaternionMsgFromYaw ( yt_[0].theta() );

    cv::Matx<double, 3, 3> C_X2 = cv::Mat_<double> ( C_Yt_, cv::Range ( 0, 3 ), cv::Range ( 0, 3 ) );
    xt_.marker.covariance[6*0 + 0] = C_X2(0, 0);
    xt_.marker.covariance[6*0 + 1] = C_X2(0, 1);
    xt_.marker.covariance[6*0 + 5] = C_X2(0, 2);
    xt_.marker.covariance[6*1 + 0] = C_X2(1, 0);
    xt_.marker.covariance[6*1 + 1] = C_X2(1, 1);
    xt_.marker.covariance[6*1 + 5] = C_X2(1, 2);
    xt_.marker.covariance[6*5 + 0] = C_X2(2, 0);
    xt_.marker.covariance[6*5 + 1] = C_X2(2, 1);
    xt_.marker.covariance[6*5 + 5] = C_X2(2, 2);

    pub_xt_.publish ( xt_ );

    // publish estimated landmark poses and their variance
    mt_.header.stamp = ros::Time::fromBoost ( slam_technique_->time_last_update() );
    mt_.header.seq++;

    mt_.markers.resize ( yt_.size() - 1 );
    for ( size_t i = 0; i < mt_.markers.size(); i++ ) {
        mt_.markers[i].marker.pose.position.x = yt_[i+1].x();
        mt_.markers[i].marker.pose.position.y = yt_[i+1].y();
        mt_.markers[i].marker.pose.position.z = 0;
        mt_.markers[i].marker.pose.orientation = tf::createQuaternionMsgFromYaw ( yt_[i+1].theta() );

        cv::Matx<double, 3, 3> C_Mi2 = cv::Mat_<double> ( C_Yt_, cv::Range ( 3* ( i+1 ), 3* ( i+1 ) + 3 ), cv::Range ( 3* ( i+1 ), 3* ( i+1 ) + 3 ) );
        mt_.markers[i].covariance[6*0 + 0] = C_Mi2(0, 0);
        mt_.markers[i].covariance[6*0 + 1] = C_Mi2(0, 1);
        mt_.markers[i].covariance[6*0 + 5] = C_Mi2(0, 2);
        mt_.markers[i].covariance[6*1 + 0] = C_Mi2(1, 0);
        mt_.markers[i].covariance[6*1 + 1] = C_Mi2(1, 1);
        mt_.markers[i].covariance[6*1 + 5] = C_Mi2(1, 2);
        mt_.markers[i].covariance[6*5 + 0] = C_Mi2(2, 0);
        mt_.markers[i].covariance[6*5 + 1] = C_Mi2(2, 1);
        mt_.markers[i].covariance[6*5 + 5] = C_Mi2(2, 2);
    }

    pub_mt_.publish ( mt_ );
}

/**
 * copies incoming robot command message
 * @param cmd
 **/
void SLAMNode::callbackCmd ( const geometry_msgs::Twist& cmd ) {
    ut_.v() = cmd.linear.x;
    ut_.w() = cmd.angular.z;
}

/**
 * copies incoming marker messages to the base class
 * @param marker
 **/
void SLAMNode::callbackMarker ( const marker_msgs::MarkerDetection &_marker ) {
    assert ( zt_->getType() == tuw::Measurement::Type::MARKER );
    MeasurementMarkerPtr zt = std::static_pointer_cast<MeasurementMarker> ( zt_ );

    try {
        tf::StampedTransform transform;
        tf_listener_->lookupTransform ( tf::resolve ( n_.getNamespace(),"base_link" ), _marker.header.frame_id, ros::Time ( 0 ), transform );
        zt->pose2d() = Pose2D ( transform.getOrigin().getX(),
                                       transform.getOrigin().getY(),
                                       tf::getYaw ( transform.getRotation() ) );
    } catch ( tf::TransformException &ex ) {
        ROS_ERROR ( "[%s callbackMarker] %s", ros::this_node::getName().c_str(), ex.what() );
        zt->pose2d() = Pose2D ( 0.225,  0, 0 );
    }

    if ( ( _marker.view_direction.x == 0 ) && ( _marker.view_direction.y == 0 ) && ( _marker.view_direction.z == 0 ) && ( _marker.view_direction.w == 1 ) ) {
        zt->angle_min() = -_marker.fov_horizontal/2.;
        zt->angle_max() = _marker.fov_horizontal/2.;
    } else {
        ROS_ERROR ( "[%s callbackMarker] %s", ros::this_node::getName().c_str(), "This node only deals with straight forward looking view directions" );
    }
    zt->range_min() = _marker.distance_min;
    zt->range_max() = _marker.distance_max;
    zt->range_max_id() = _marker.distance_max_id;
    zt->stamp() = _marker.header.stamp.toBoost();
    zt->resize ( _marker.markers.size() );

    for ( size_t i = 0; i < zt->size(); i++ ) {
        tf::Vector3 v;
        tf::pointMsgToTF ( _marker.markers[i].pose.position, v );
        double orientation = tf::getYaw ( _marker.markers[i].pose.orientation );

        zt->operator[] ( i ).ids = _marker.markers[i].ids;
        zt->operator[] ( i ).ids_confidence = _marker.markers[i].ids_confidence;
        zt->operator[] ( i ).length = v.length();
        zt->operator[] ( i ).angle = atan2 ( v.getY(), v.getX() );
        zt->operator[] ( i ).orientation = orientation;
        zt->operator[] ( i ).pose = Pose2D ( v.getX(), v.getY(), orientation );
    }
}

/**
 * copies incoming odemetry messages to the base class
 * @param odom
 **/
void SLAMNode::callbackGroundTruth ( const nav_msgs::Odometry& ground_truth ) {
    tf::Quaternion q;
    double roll = 0, pitch = 0, yaw = 0;

    tf::quaternionMsgToTF ( ground_truth.pose.pose.orientation, q );
    tf::Matrix3x3 ( q ).getRPY ( roll, pitch, yaw );

    pose_ground_truth_.set ( ground_truth.pose.pose.position.x, ground_truth.pose.pose.position.y, yaw );
}

void SLAMNode::callbackConfigSLAM ( tuw_marker_slam::SLAMConfig &config, uint32_t level ) {
    ROS_INFO ( "callbackConfigSLAM!" );
    config_ = config;
}

void SLAMNode::callbackConfigEKFSLAM ( tuw_marker_slam::EKFSLAMConfig &config, uint32_t level ) {
    ROS_INFO ( "callbackConfigEKFSLAM!" );
    slam_technique_->setConfig ( &config );
}
