#ifndef TUW_MARKER_SLAM_NODE_H
#define TUW_MARKER_SLAM_NODE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <marker_msgs/MarkerDetection.h>
#include <marker_msgs/MarkerWithCovarianceArray.h>
#include <dynamic_reconfigure/server.h>

#include "tuw_marker_slam/tuw_marker_slam.h"
#include "tuw_marker_slam/SLAMConfig.h"
#include "tuw_marker_slam/EKFSLAMConfig.h"

/**
 * class to cover the ros communication for the slam
 **/
class SLAMNode : public tuw::SLAM {
public:
    SLAMNode ( ros::NodeHandle & n );                                                                           /// constructor
    void cycle ();                                                                                              /// triggers the SLAM cycle
    void publish ();                                                                                            /// publishes the estimated landmark map and the estimated vehicle pose in it
private:
    ros::NodeHandle n_;                                                                                         /// node handler to the root node
    ros::NodeHandle n_param_;                                                                                   /// node handler to the current node

    ros::Subscriber sub_cmd_;                                                                                   /// subscriber to the command
    ros::Subscriber sub_marker_;                                                                                /// subscriber to the marker detector

    ros::Publisher pub_xt_;                                                                                     /// publisher for the estimated vehicle pose and its covariance
    ros::Publisher pub_mt_;                                                                                     /// publisher for the estimated landmark poses and their covariances
    geometry_msgs::PoseWithCovarianceStamped xt_;                                                               /// estimated vehicle pose and its covariance to publish
    marker_msgs::MarkerWithCovarianceArray mt_;                                                                 /// estimated landmark poses and their covariances to publish

    tf::TransformBroadcaster tf_broadcaster_;                                                                   /// broadcasts transformation messages
    std::shared_ptr<tf::TransformListener> tf_listener_;                                                        /// listener to receive transformation messages
    std::string frame_id_map_;                                                                                  /// frame id of map (for transformations)
    std::string frame_id_odom_;                                                                                 /// frame id of odom (for transformations)
    std::string frame_id_base_;                                                                                 /// frame id of base (for transformations)

    void callbackCmd ( const geometry_msgs::Twist& );                                                           /// callback function to catch motion commands
    void callbackMarker ( const marker_msgs::MarkerDetection& );                                                /// callback function to catch incoming marker data

    dynamic_reconfigure::Server<tuw_marker_slam::SLAMConfig> reconfigureServerSLAM_;                            /// parameter server stuff general use
    dynamic_reconfigure::Server<tuw_marker_slam::SLAMConfig>::CallbackType reconfigureFncSLAM_;                 /// parameter server stuff general use
    void callbackConfigSLAM ( tuw_marker_slam::SLAMConfig &config, uint32_t level );                            /// callback function on incoming parameter changes for general use

    std::shared_ptr<dynamic_reconfigure::Server<tuw_marker_slam::EKFSLAMConfig> > reconfigureServerEKFSLAM_;    /// parameter server stuff for the EKF SLAM
    dynamic_reconfigure::Server<tuw_marker_slam::EKFSLAMConfig>::CallbackType reconfigureFncEKFSLAM_;           /// parameter server stuff for the EKF SLAM
    void callbackConfigEKFSLAM ( tuw_marker_slam::EKFSLAMConfig &config, uint32_t level );                      /// callback function on incoming parameter changes for the EKF SLAM
};

#endif // TUW_MARKER_SLAM_NODE_H
