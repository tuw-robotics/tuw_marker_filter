#ifndef TUW_MARKER_SLAM_NODE_H
#define TUW_MARKER_SLAM_NODE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <marker_msgs/MarkerDetection.h>
#include <marker_msgs/MarkerWithCovarianceStamped.h>
#include <marker_msgs/MarkerWithCovarianceArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>

#include "tuw_marker_slam/tuw_marker_slam.h"
#include "tuw_marker_slam/SLAMConfig.h"
#include "tuw_marker_slam/EKFSLAMConfig.h"

/**
 * class to cover the ros communication for the self-localization
 **/
class SLAMNode : public tuw::SLAM {
public:
    SLAMNode ( ros::NodeHandle & n );                                                                   /// Constructor
    void cycle ();                                                                                      /// tiggers the SLAM cycle
    void publish ();                                                                                    /// publishes the estimated robot pose and mapping
private:
    ros::NodeHandle n_;                                                                                 /// node handler to the root node
    ros::NodeHandle n_param_;                                                                           /// node handler to the current node
    ros::Subscriber sub_cmd_;                                                                           /// Subscriber to the command measurements
    ros::Subscriber sub_marker_;                                                                        /// Subscriber to the marker detector
    ros::Subscriber sub_ground_truth_;                                                                  /// Subscriber to the ground truth pose (simulation only)
    ros::Publisher pub_xt_;                                                                             /// publisher for the estimated robot pose and its variance
    ros::Publisher pub_mt_;                                                                             /// publisher for the estimated landmark poses and their variances
    tf::TransformBroadcaster tf_broadcaster_;                                                           /// broadcasts transformation messages
    std::shared_ptr<tf::TransformListener> tf_listener_;                                                /// listener to receive transformation messages -> to get the marker detector pose
    marker_msgs::MarkerWithCovarianceStamped xt_;                                                       /// estimated robot pose and its variance to publish
    marker_msgs::MarkerWithCovarianceArray mt_;                                                         /// estimated landmark poses and their variances to publish

    tuw::Pose2D pose_ground_truth_;                                                                     /// ground truth state x, y, alpha (simulation only)
    tuw::Pose2D origin_;                                                                                /// origin x, y, alpha of the slam created map in the world (simulation only)

    void callbackCmd ( const geometry_msgs::Twist& );                                                   /// callback function to catch motion commands
    void callbackMarker ( const marker_msgs::MarkerDetection& );                                        /// callback function to catch incoming sensor data
    void callbackGroundTruth ( const nav_msgs::Odometry& );                                             /// callback function to catch ground truth pose messages

    dynamic_reconfigure::Server<tuw_marker_slam::SLAMConfig> reconfigureServerSLAM_;                           /// parameter server stuff general use
    dynamic_reconfigure::Server<tuw_marker_slam::SLAMConfig>::CallbackType reconfigureFncSLAM_;                /// parameter server stuff general use
    void callbackConfigSLAM ( tuw_marker_slam::SLAMConfig &config, uint32_t level );                           /// callback function on incoming parameter changes for general use

    std::shared_ptr<dynamic_reconfigure::Server<tuw_marker_slam::EKFSLAMConfig> > reconfigureServerEKFSLAM_;   /// parameter server stuff for the EKF SLAM
    dynamic_reconfigure::Server<tuw_marker_slam::EKFSLAMConfig>::CallbackType reconfigureFncEKFSLAM_;          /// parameter server stuff for the EKF SLAM
    void callbackConfigEKFSLAM ( tuw_marker_slam::EKFSLAMConfig &config, uint32_t level );                     /// callback function on incoming parameter changes for the EKF SLAM
};

#endif // TUW_MARKER_SLAM_NODE_H
