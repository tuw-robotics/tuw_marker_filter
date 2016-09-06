#ifndef EKF_SLAM_H
#define EKF_SLAM_H

#include "tuw_marker_slam/slam_technique.h"
#include "tuw_marker_slam/measurement_marker.h"
#include "tuw_marker_slam/EKFSLAMConfig.h"

namespace tuw {

class EKFSLAM;
typedef std::shared_ptr< EKFSLAM > EKFSLAMPtr;
typedef std::shared_ptr< EKFSLAM const> EKFSLAMConstPtr;

/**
 * extended kalman filter slam
 */
class EKFSLAM : public SLAMTechnique {
public:
    /**
    * Constructor
    *
    * @param beta parameters for the implemented measurement noise model
    **/
    EKFSLAM( const std::vector<double> beta );

    /**
     * starts the SLAM cycle and predicts the robot and landmark poses at the timestamp
     * encoded into the measurement
     *
     * @param yt implicit return of the combined state yt = (xt, mt_1, ..., mt_n) with xt = (x, y, alpha), mt_i = (x_i, y_i, alpha_i)
     * @param C_Yt implicit return of the combined covariance matrix of combined state
     * @param ut current control command
     * @param zt measurment with a timestamp
     **/
    void cycle ( std::vector<Pose2D> &yt, cv::Mat_<double> &C_Yt, const Command &ut, const MeasurementConstPtr &zt );

    /**
     * virtual function to set the config parameters
     *
     * @param config of type tuw_marker_slam::EKFSLAMConfig*
     **/
    void setConfig ( const void *config );
private:
    /**
     * Different supported data association modes
     **/
    enum DataAssociationMode {
        ID = 0,
        NNSF_LOCAL = 1,
        NNSF_GLOBAL = 2,
    };

    /**
     * Different supported update modes
     **/
    enum UpdateMode {
        None = 0,
        Single = 1,
        Combined = 2,
    };

    /**
    * struct to represent correspondeces and their data
    **/
    struct CorrData {
        std::pair<size_t, size_t> ij;           /// i >= 0, j > 0...landmark j corressponds to measurement i (else no correspondence)
        cv::Matx<double, 3, 3> Q;               /// measurement noise
        cv::Vec<double, 3> v;                   /// difference of obtained and predicted measurement
        cv::Matx<double, 3, 3> dx;              /// x deviation in H_ij = (dx 0 .. 0 dm 0 .. 0)
        cv::Matx<double, 3, 3> dm;              /// m deviation in H_ij = (dx 0 .. 0 dm 0 .. 0)
        cv::Matx<double, 3, 3> S_inv;           /// inverted matrix used amongst other for the mahalanobis distance
    };
    typedef std::shared_ptr< CorrData > CorrDataPtr;
    typedef std::shared_ptr< CorrData const > CorrDataConstPtr;

    /**
     * initializes the filter
     **/
    void init();

    /**
     * predicting robots pose
     * @param ut command at time t
     **/
    void prediction ( const Command &ut );

    /**
     * data association to find matching between measurements and landmarks
     *
     * @param zt measurements at time t
     **/
    void data_association ( const MeasurementMarkerConstPtr &zt );

    /**
     * performs Nearest Neighbor Standard Filter (NNSF) by associating each measurement to
     * an up then unassociated landmark with minimum Mahalanobis distance
     * 
     * Note: does not avoid local optimums, i.e. to a single landmark multiple measurements
     * could be assigned. In this case both associations are deprecated (conservative approach)
     *
     * @param zt measurements at time t
     * @param gamma gamma is a threshold such that 100*(1-alpha)% of true measurements are rejected
     *              (with alpha from the config)
     **/
    void NNSF_local ( const MeasurementMarkerConstPtr &zt, const double gamma );

    /**
     * performs Nearest Neighbor Standard Filter (NNSF) by finding a minimum Mahalanobis distance
     * assignment between measurements and unassociated landmarks
     * 
     * @param zt measurements at time t
     * @param gamma gamma is a threshold such that 100*(1-alpha)% of true measurements are rejected
     *              (with alpha from the config)
     **/
    void NNSF_global ( const MeasurementMarkerConstPtr &zt, const double gamma );

    /**
     * compares observed and predicted measurement
     * 
     * @param zt measurements at time t
     * @param corr contains the indices of the measurement and associated landmark, also used for implicit return
     **/
    void measurement ( const MeasurementMarkerConstPtr &zt, const CorrDataPtr &corr );

    /**
     * implements the measurement noise model
     * 
     * @param zi single measured marker
     * @return covariance matrix describing the measurement noise
     **/
    cv::Matx<double, 3, 3> measurement_noise ( const MeasurementMarker::Marker zi );

    /**
     * correcting robot and landmarks poses
     *
     * @param zt measurements at time t
     **/
    void update();

    /**
     * iterates over each single observation and corrects the robot and landmarks poses
     **/
    void update_single();

    /**
     * corrects the robot and landmarks poses by using all observations combined in one step
     **/
    void update_combined();

    /**
     * integrating new landmarks
     **/
    void integration ( const MeasurementMarkerConstPtr &zt );

    tuw_marker_slam::EKFSLAMConfig config_;     /// parameters
    const std::vector<double> beta_;            /// parameters for the implemented measurement noise model

    cv::Mat_<double> y;                         /// mean vector of y = (x m1 m2 ...)
    cv::Mat_<double> x;                         /// mean vector of x
    cv::Mat_<double> C_Y;                       /// covariance matrix of y = (x m1 m2 ...)
    cv::Mat_<double> C_X;                       /// covariance matrix of x

    std::map<int, size_t> f_kj;                 /// correspondences: f[k] = j <-> marker k corressponds to landmark j

    std::vector<CorrDataPtr> c_ij;              /// measurement to landmark correspondences
    std::vector<CorrDataPtr> c_ji;              /// landmark to measurement correspondences (ignore j = 0 since landmark numbering starts with 1)

    std::vector<size_t> z_known;                /// measurements corresponding to known landmarks
    std::vector<size_t> z_new;                  /// measurements corresponding to new landmarks
};
};

#endif // EKF_SLAM_H
