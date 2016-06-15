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
    int mode;

    // reads shared parameter on the operation mode
    n_param_.getParam ( "mode", mode );
    switch ( mode ) {
    case SLAMTechnique::EKF:
        zt_ = std::make_shared<tuw::MeasurementMarker>();
        slam_technique_ = std::make_shared<tuw::EKFSLAM>();
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
    pub_xt_ = n_param_.advertise<geometry_msgs::PoseStamped> ( "xt", 1 );
    pub_xt_var_ = n_param_.advertise<visualization_msgs::Marker> ( "xt_var", 1 );
    xt_.header.seq = 0;
    xt_var_.header.seq = 0;
    n_param_.param<std::string> ( "frame_id_map", xt_.header.frame_id, "map" );
    n_param_.param<std::string> ( "frame_id_map", xt_var_.header.frame_id, "map" );

    /// defines publishers for the resulting landmark poses
    pub_mt_ = n_param_.advertise<geometry_msgs::PoseArray> ( "mt", 1 );
    pub_mt_var_ = n_param_.advertise<visualization_msgs::MarkerArray> ( "mt_var", 1 );

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

    // publish estimated robot pose
    xt_.header.stamp = ros::Time::fromBoost ( slam_technique_->time_last_update() );
    xt_.header.seq++;
    xt_.pose.position.x = yt_[0].x();
    xt_.pose.position.y = yt_[0].y();
    xt_.pose.position.z = 0;
    xt_.pose.orientation = tf::createQuaternionMsgFromYaw ( yt_[0].theta() );
    pub_xt_.publish ( xt_ );

    // calculate eigenvalues and eigenvectors for variance representation
    cv::Matx<double, 2, 2> C_X = cv::Mat_<double> ( C_Yt_, cv::Range ( 0, 2 ), cv::Range ( 0, 2 ) );
    cv::Mat_<double> eigval, eigvec;
    cv::eigen ( C_X, eigval, eigvec );
    double alpha = atan2 ( eigvec[0][1], eigvec[0][0] );

    // publish variance of estimated robot pose
    xt_var_.header.stamp = ros::Time::fromBoost ( slam_technique_->time_last_update() );
    xt_var_.header.seq++;
    xt_var_.ns = n_param_.getNamespace();
    xt_var_.id = 0;
    xt_var_.type = visualization_msgs::Marker::SPHERE;
    xt_var_.action = visualization_msgs::Marker::MODIFY;
    xt_var_.pose.position.x = yt_[0].x();
    xt_var_.pose.position.y = yt_[0].y();
    xt_var_.pose.position.z = 0.0;
    xt_var_.pose.orientation = tf::createQuaternionMsgFromYaw ( alpha );
    xt_var_.scale.x = 2*sqrt ( eigval[0][0] );
    xt_var_.scale.y = 2*sqrt ( eigval[1][0] );
    xt_var_.scale.z = 0.1;
    xt_var_.color.r = 1.0;
    xt_var_.color.g = 1.0;
    xt_var_.color.b = 0.0;
    xt_var_.color.a = 1.0;
    xt_var_.lifetime = ros::Duration ( ros::Rate ( 5 ) );
    pub_xt_var_.publish ( xt_var_ );

    mt_.poses.resize ( yt_.size() - 1 );
    mt_.header.stamp = ros::Time::fromBoost ( slam_technique_->time_last_update() );
    n_param_.param<std::string> ( "frame_id_map", mt_.header.frame_id, "map" );
    mt_var_.markers.resize ( yt_.size() - 1 );
    for ( size_t i = 0; i < mt_.poses.size(); i++ ) {
        // publish estimated landmark poses
        mt_.poses[i].position.x = yt_[i+1].x();
        mt_.poses[i].position.y = yt_[i+1].y();
        mt_.poses[i].position.z = 0;
        mt_.poses[i].orientation = tf::createQuaternionMsgFromYaw ( yt_[i+1].theta() );

        // calculate eigenvalues and eigenvectors for variance representation
        cv::Matx<double, 2, 2> C_Mi = cv::Mat_<double> ( C_Yt_, cv::Range ( 3* ( i+1 ), 3* ( i+1 ) + 2 ), cv::Range ( 3* ( i+1 ), 3* ( i+1 ) + 2 ) );
        cv::eigen ( C_Mi, eigval, eigvec );
        alpha = atan2 ( eigvec[0][1], eigvec[0][0] );

        // publish variance of landmark poses
        mt_var_.markers[i].header.stamp = ros::Time::fromBoost ( slam_technique_->time_last_update() );
        n_param_.param<std::string> ( "frame_id_map", mt_var_.markers[i].header.frame_id, "map" );
        mt_var_.markers[i].ns = n_param_.getNamespace();
        mt_var_.markers[i].id = i+1;
        mt_var_.markers[i].type = visualization_msgs::Marker::SPHERE;
        mt_var_.markers[i].action = visualization_msgs::Marker::ADD;
        mt_var_.markers[i].pose.position.x = yt_[i+1].x();
        mt_var_.markers[i].pose.position.y = yt_[i+1].y();
        mt_var_.markers[i].pose.position.z = 0.0;
        mt_var_.markers[i].pose.orientation = tf::createQuaternionMsgFromYaw ( alpha );
        mt_var_.markers[i].scale.x = 2*sqrt ( eigval[0][0] );
        mt_var_.markers[i].scale.y = 2*sqrt ( eigval[1][0] );
        mt_var_.markers[i].scale.z = 0.1;
        mt_var_.markers[i].color.r = 0.0;
        mt_var_.markers[i].color.g = 0.0;
        mt_var_.markers[i].color.b = 1.0;
        mt_var_.markers[i].color.a = 1.0;
        mt_var_.markers[i].lifetime = ros::Duration ( ros::Rate ( 5 ) );
    }
    pub_mt_.publish ( mt_ );
    pub_mt_var_.publish ( mt_var_ );
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

        zt->operator[] ( i ).id = _marker.markers[i].ids[0];
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
