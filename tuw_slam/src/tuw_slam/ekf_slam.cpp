#include "tuw_slam/ekf_slam.h"
#include "tuw_slam/munkre.h"
#include <boost/math/distributions/chi_squared.hpp>

using namespace tuw;

EKFSLAM::EKFSLAM() :
    SLAMTechnique ( EKF ) {
}

void EKFSLAM::init() {
    reset_ = false;

    // initial mean
    y = cv::Mat_<double> ( 3, 1, 0.0 );

    // initial covariance
    C_Y = cv::Mat_<double> ( 3, 3, 0.0 );

    // initialize covariance submatrices pointing to the same data (no copy!)
    x = cv::Mat_<double> ( y, cv::Range ( 0, 3 ), cv::Range ( 0, 1 ) );
    C_X = cv::Mat_<double> ( C_Y, cv::Range ( 0, 3 ), cv::Range ( 0, 3 ) );

    // reset fiducial landmark correspondences
    f_kj.clear();
}

void EKFSLAM::cycle ( std::vector<Pose2D> &yt, cv::Mat_<double> &C_Yt, const Command &ut, const MeasurementConstPtr &zt ) {
    if ( reset_ ) init();
    if ( zt->getType() == tuw::Measurement::Type::FIDUCIAL && updateTimestamp ( zt->stamp() ) ) {
        MeasurementFiducialConstPtr z = std::static_pointer_cast<MeasurementFiducial const> ( zt );

        // control noise
        M = cv::Matx<double, 2, 2> ( config_.alpha_1*ut.v() *ut.v() + config_.alpha_2*ut.w() *ut.w(), 0,
                                     0, config_.alpha_3*ut.v() *ut.v() + config_.alpha_4*ut.w() *ut.w() );

        // measurement noise
        Q = cv::Matx<double, 3,3> ( z->sigma_radial() * z->sigma_radial(), 0, 0,
                                    0, z->sigma_azimuthal() * z->sigma_azimuthal(), 0,
                                    0, 0, z->sigma_yaw() * z->sigma_yaw() );

        // invariance check
        assert ( y.rows % 3 == 0 && y.cols == 1 );
        assert ( x.rows == 3 && x.cols == 1 );
        assert ( C_Y.rows == C_Y.cols && C_Y.rows == y.rows );
        assert ( C_X.rows == C_X.cols && C_X.rows == x.rows );

        prediction ( ut );
        data_association ( z );
        update();
        integration ( z );
    }

    // implicit return
    yt.resize ( y.rows/3 );
    for ( size_t i = 0; i < yt.size(); i++ ) {
        yt[i].x() = y[3*i + 0][0];
        yt[i].y() = y[3*i + 1][0];
        yt[i].theta() = y[3*i + 2][0];
    }
    C_Yt = C_Y;
}

void EKFSLAM::setConfig ( const void *config ) {
    config_ = * ( ( tuw_slam::EKFSLAMConfig* ) config );
}

void EKFSLAM::prediction ( const Command &ut ) {
    if ( !config_.enable_prediction ) return;

    // pre-calculate needed data
    double dt = duration_last_update_.total_microseconds() /1000000.0;
    double r = ut.v() /ut.w();
    double s = sin ( x[2][0] );
    double c = cos ( x[2][0] );
    double s_dt = sin ( x[2][0] + ut.w() *dt );
    double c_dt = cos ( x[2][0] + ut.w() *dt );

    // calculate update and its derivatives (= jacobian matrices)
    cv::Vec<double, 3> dx_;
    cv::Matx<double, 3, 3> G_x_;
    cv::Matx<double, 3, 2> G_u;
    if ( std::abs ( ut.w() ) > __DBL_MIN__ ) {
        dx_ = cv::Vec<double, 3> ( -r*s + r*s_dt,
                                   r*c - r*c_dt,
                                   ut.w() *dt );
        G_x_ = cv::Matx<double, 3, 3> ( 1, 0, -r*c + r*c_dt,
                                        0, 1, -r*s + r*s_dt,
                                        0, 0, 1 );
        G_u = cv::Matx<double, 3, 2> ( ( -s + s_dt ) /ut.w(),  r * ( s - s_dt ) / ut.w() + r*c_dt*dt,
                                       ( c - c_dt ) /ut.w(), -r * ( c - c_dt ) / ut.w() + r*s_dt*dt,
                                       0, dt );
    } else {
        dx_ = cv::Vec<double, 3> ( ut.v() *dt*c,
                                   ut.v() *dt*s,
                                   0 );
        G_x_ = cv::Matx<double, 3, 3> ( 1, 0, -ut.v() *dt*s,
                                        0, 1,  ut.v() *dt*c,
                                        0, 0, 1 );
        G_u = cv::Matx<double, 3, 2> ( dt*c, -0.5*dt*dt*ut.v() *s,
                                       dt*s, 0.5*dt*dt*ut.v() *c,
                                       0, dt );
    }

    // convert from Matx/Vec to Mat_ in order to use +/* operations on submatrices
    cv::Mat_<double> dx = cv::Mat_<double> ( dx_ );
    cv::Mat_<double> G_x = cv::Mat_<double> ( G_x_ );
    cv::Mat_<double> R = cv::Mat_<double> ( G_u * M * G_u.t() );

    // update mean and covariance
    x = x + dx;
    C_X = G_x*C_X*G_x.t() + R;
    for ( size_t i = 3; i < C_Y.cols; i += 3 ) {
        // references to submatrices
        cv::Mat_<double> C_XM_i = cv::Mat_<double> ( C_Y, cv::Range ( 0, 3 ), cv::Range ( i, i + 3 ) );
        cv::Mat_<double> C_MX_i = cv::Mat_<double> ( C_Y, cv::Range ( i, i + 3 ), cv::Range ( 0, 3 ) );

        // calculation & matrix update
        C_XM_i = G_x*C_XM_i;
        C_MX_i = C_XM_i.t();
    }

    // normalize angle
    x[2][0] = angle_normalize ( x[2][0] );
}

void EKFSLAM::data_association ( const MeasurementFiducialConstPtr &zt ) {
    // initialization
    c_ij = std::vector<CorrDataPtr> ( zt->size() );
    c_ji = std::vector<CorrDataPtr> ( y.rows/3 );
    z_known.clear();
    z_new.clear();

    // gamma is a threshold such that 100*(1-alpha)% of true measurements are rejected
    boost::math::chi_squared chi_squared = boost::math::chi_squared ( 3 );
    double gamma = boost::math::quantile(chi_squared, config_.alpha);

    // find correspondences based on measurement fiducical IDs
    for ( size_t i = 0; i < zt->size(); i++ ) {
        assert ( zt->operator[] ( i ).id >= 0 );

        if ( zt->operator[] ( i ).id == 0 ) {
            // invalid measurement
        } else {
            // valid measurement
            std::map<size_t, size_t>::iterator it = f_kj.find ( zt->operator[] ( i ).id );

            if ( it != f_kj.end() ) {
                // known landmark j
                size_t j = it->second;
                assert ( 0 < j && j < c_ji.size() );

                // create correspondence data
                CorrDataPtr c = std::make_shared<CorrData>();
                c->ij = std::pair<size_t, size_t> ( i, j );
                measurement ( zt, c );

                // document founded correspondence
                assert ( c_ij[i] == nullptr && c_ji[j] == nullptr );
                c_ij[i] = c;
                c_ji[j] = c;
                z_known.push_back(i);
            } else {
                // new landmark
                z_new.push_back( i );
            }
        }
    }
    
    // complement correspondences based on probability
    switch (config_.data_association_mode) {
        case ID:
            // Nothing to do anymore
            break;
        case NNSF_LOCAL:
            NNSF_local ( zt, gamma );
            break;
        case NNSF_GLOBAL:
            NNSF_global ( zt, gamma );
            break;
        default:
            assert ( false );
    }
}

void EKFSLAM::NNSF_local ( const MeasurementFiducialConstPtr &zt, const double gamma ) {
    std::vector<size_t> m_reject;

    // find correspondences based on Mahalanobis distance
    for ( size_t i = 0; i < c_ij.size(); i++ ) {
        assert ( c_ij[i] == nullptr || c_ij[i]->ij.first == i );

        // consider only measurements without fiducial IDs
        if ( zt->operator[] ( i ).id > 0) continue;
        assert ( c_ij[i] == nullptr );

        double min_d_2 = std::numeric_limits<double>::infinity();
        CorrDataPtr min_c = nullptr;
        for ( size_t j = 1; j < c_ji.size(); j++ ) {
            assert ( c_ji[j] == nullptr || c_ji[j]->ij.second == j );

            // consider only landmarks without correspondences based on measurement fiducial IDs
            if ( c_ji[j] != nullptr && zt->operator[] ( c_ji[j]->ij.first ).id > 0 ) continue;

            // create correspondence data
            CorrDataPtr c = std::make_shared<CorrData>();
            c->ij = std::pair<size_t, size_t> ( i, j );
            measurement ( zt, c );

            // calculate Mahalanobis distance: d = sqrt(v^T S^(-1) v)
            double d_2 = ( c->v.t() * c->S_inv * c->v ) [0];
            if (d_2 < min_d_2) {
                min_d_2 = d_2;
                min_c = c;
            }
        }
        
        // update correspondences
        if ( min_c == nullptr || min_d_2 > gamma ) continue;

        // new (possible) correspondence found
        size_t min_i = min_c->ij.first;
        size_t min_j = min_c->ij.second;
        assert ( i == min_i && 0 < min_j && min_j < c_ji.size() );

        if ( c_ji[min_j] != nullptr ) {
            // landmark already correspond to another measurement
            size_t old_i = c_ji[min_j]->ij.first;
            ROS_INFO ( "measurements %lu and %lu correspond both to landmark %lu", old_i, min_i, min_j );
            if ( zt->operator[] ( old_i ).id > 0 ) {
                // old correspondence is fix -> reject current measurement
                ROS_INFO ( "reject measurement %lu", min_i );
            } else {
                // old correspondence is just probably -> reject both measurements
                ROS_INFO ( "reject measurement %lu (conservative approach)", i );

                if ( c_ij[old_i] != nullptr ) {
                    ROS_INFO ( "reject measurement %lu (conservative approach)", old_i );

                    // remove already added old measurement from known measurements
                    std::vector<size_t>::iterator it = std::find ( z_known.begin(), z_known.end(), old_i );
                    assert ( it != z_known.end() );
                    z_known.erase( it );
                    
                    // reset old correspondence c_ij at the first time and
                    // remember old correspondence c_ji for later reset
                    c_ij[old_i] = nullptr;
                    m_reject.push_back( min_j );
                }
            }
        } else {
            // document founded correspondence
            assert ( c_ij[min_i] == nullptr && c_ji[min_j] == nullptr );
            c_ij[min_i] = min_c;
            c_ji[min_j] = min_c;
            z_known.push_back( min_i );
        }
    }

    // reset correspondences c_ji found but invalidated again during corresponde update
    for (auto j: m_reject) c_ji[j] = nullptr;
}

void EKFSLAM::NNSF_global ( const MeasurementFiducialConstPtr &zt, const double gamma ) {
    // book keeping: measurements
    std::vector<size_t> map_i;
    for ( size_t i = 0; i < c_ij.size(); i++ ) {
        assert ( c_ij[i] == nullptr || c_ij[i]->ij.first == i );

        // consider only measurements without fiducial IDs
        if ( zt->operator[] ( i ).id > 0 ) continue;
        assert ( c_ij[i] == nullptr );

        map_i.push_back(i);
    }

    // no further correspondences possible
    if (map_i.size() == 0) return;

    // book keeping: landmarks
    std::vector<size_t> map_j;
    for ( size_t j = 1; j < c_ji.size(); j++ ) {
        assert ( c_ji[j] == nullptr || c_ji[j]->ij.second == j );

        // consider only landmarks without correspondences based on measurement fiducial IDs
        assert ( c_ji[j] == nullptr || zt->operator[] ( c_ji[j]->ij.first ).id > 0);
        if ( c_ji[j] != nullptr ) continue;

        map_j.push_back(j);
    }

    // no further correspondences possible
    if (map_j.size() == 0) return;

    // build assignment matrix (Mahalanobis distance)
    std::vector<std::vector<CorrDataPtr>> C = std::vector<std::vector<CorrDataPtr>> ( map_i.size() );
    cv::Mat_<double> D_2 = cv::Mat_<double> ( map_i.size(), map_j.size() );
    for ( size_t x = 0; x < map_i.size(); x++ ) {
        C[x] = std::vector<CorrDataPtr> ( map_j.size() );
        for ( size_t y = 0; y < map_j.size(); y++ ) {
            // create correspondence data
            CorrDataPtr c = std::make_shared<CorrData>();
            c->ij = std::pair<size_t, size_t> ( map_i[x], map_j[y] );
            measurement ( zt, c );

            // calculate Mahalanobis distance: d = sqrt(v^T S^(-1) v)
            double d_2 = ( c->v.t() * c->S_inv * c->v ) [0];
            assert ( d_2 >= 0 );

            // book keeping: correspondence data
            C[x][y] = c;
            D_2[x][y] = d_2;
        }
    }

    // update correspondences   
    std::vector<std::pair<size_t, size_t>> assignment = Munkre::find_minimum_assignment(D_2);
    for (size_t k = 0; k < assignment.size(); k++) {
        // book keeping measurement and landmark
        size_t x = assignment[k].first;
        size_t y = assignment[k].second;

        if ( D_2[x][y] > gamma ) continue;

        // new correspondence found
        assert ( x < C.size() && y < C[x].size() );
        size_t i = C[x][y]->ij.first;
        size_t j = C[x][y]->ij.second;
        assert ( i < c_ij.size() && 0 < j && j < c_ji.size() );

        // document founded correspondence
        assert ( c_ij[i] == nullptr && c_ji[j] == nullptr );
        c_ij[i] = C[x][y];
        c_ji[j] = C[x][y];
        z_known.push_back( i );
    }
}

void EKFSLAM::measurement ( const MeasurementFiducialConstPtr &zt, const CorrDataPtr &corr ) {
    const size_t i = corr->ij.first;
    const size_t j = corr->ij.second;
    assert ( i < zt->size() && 0 < j && j < y.rows/3 );

    // transformation matrix from robot coordinates in global coordinates
    cv::Matx<double, 4, 4> T_x = Pose2D ( x[0][0], x[1][0], x[2][0] ).tf2();

    // measurement prediction of landmark j
    cv::Vec<double, 4> sensor = T_x * zt->getSensorPose().hv();
    cv::Vec<double, 2> delta = cv::Vec<double, 2> ( y[3*j + 0][0] - sensor[0],
                               y[3*j + 1][0] - sensor[1] );
    double q = ( delta.t() * delta ) [0];

    // predicted measurement
    cv::Vec<double, 3> z_ = cv::Vec<double, 3> ( sqrt ( q ),
                            angle_difference ( atan2 ( delta[1], delta[0] ), sensor[2] ),
                            angle_difference ( y[3*j + 2][0], sensor[2] ) );

    // obtained measurement
    cv::Vec<double, 3> z = cv::Vec<double, 3> ( zt->operator[] ( i ).length,
                           zt->operator[] ( i ).angle ,
                           zt->operator[] ( i ).orientation );

    // compare predicted and obtainend measurement
    corr->v[0] = z[0] - z_[0];
    corr->v[1] = angle_difference ( z[1], z_[1] );
    corr->v[2] = angle_difference ( z[2], z_[2] );

    // pre-calculate needed data
    assert ( q > 0 );
    double s = sin ( x[2][0] );
    double c = cos ( x[2][0] );
    double ddeltax =  zt->getSensorPose().x() *s + zt->getSensorPose().y() *c;
    double ddeltay = -zt->getSensorPose().x() *c + zt->getSensorPose().y() *s;
    double dx_sq = delta[0]/sqrt ( q );
    double dy_sq = delta[1]/sqrt ( q );
    double dx_q = delta[0]/q;
    double dy_q = delta[1]/q;

    //H_ij = (dx 0 .. 0 dm 0 .. 0)
    corr->dx = cv::Matx<double, 3, 3> ( -dx_sq, -dy_sq, dx_sq*ddeltax + dy_sq*ddeltay,
                                        dy_q, -dx_q, ddeltay*dx_q - ddeltax*dy_q - 1,
                                        0, 0, -1 );

    corr->dm = cv::Matx<double, 3, 3> ( dx_sq, dy_sq, 0,
                                        -dy_q, dx_q, 0,
                                        0, 0, 1 );

    // S_i = H_i*C_Y*H_i^T + Q
    cv::Matx<double, 3, 3> C_X_ = cv::Mat_<double> ( C_Y, cv::Range ( 0, 3 ), cv::Range ( 0, 3 ) );
    cv::Matx<double, 3, 3> C_M = cv::Mat_<double> ( C_Y, cv::Range ( 3*j, 3*j + 3 ), cv::Range ( 3*j, 3*j + 3 ) );
    cv::Matx<double, 3, 3> C_XM = cv::Mat_<double> ( C_Y, cv::Range ( 0, 3 ), cv::Range ( 3*j, 3*j + 3 ) );

    cv::Matx<double, 3, 3> tmp = corr->dx*C_XM*corr->dm.t();
    cv::Matx<double, 3, 3> S = corr->dx*C_X_*corr->dx.t() + tmp + tmp.t() + corr->dm*C_M*corr->dm.t() + Q;
    corr->S_inv = S.inv();
}

void EKFSLAM::update() {
    switch (config_.update_mode) {
        case None:
            // Nothing to do
            break;
        case Single:
            update_single();
            break;
        case Combined:
            update_combined();
            break;
        default:
            assert ( false );
    }
}

void EKFSLAM::update_single() {
    if ( z_known.size() == 0 ) return;

    for ( auto i: z_known ) {
        assert ( i < c_ij.size() );

        // obtained measurement i corresponds to landmark j
        int j = c_ij[i]->ij.second;
        assert ( 0 < j && j < y.rows/3 && c_ji[j] != nullptr );

        // fetch needed data
        cv::Mat_<double> v = cv::Mat_<double> ( c_ji[j]->v );
        cv::Mat_<double> S_inv = cv::Mat_<double> ( c_ji[j]->S_inv );

        // K_i = C_Y*H_i^T*S_i^(-1)
        cv::Mat_<double> K_i = cv::Mat_<double> ( C_Y.rows, 3, 0.0 );
        for ( size_t k = 0; k < K_i.rows; k += 3 ) {
            // reference to submatrix
            cv::Mat_<double> K_i_k = cv::Mat_<double> ( K_i, cv::Range ( k, k + 3 ), cv::Range ( 0, 3 ) );

            // calculation & matrix update
            cv::Matx<double, 3, 3> C_MX_k = cv::Mat_<double> ( C_Y, cv::Range ( k, k + 3 ), cv::Range ( 0, 3 ) );
            cv::Matx<double, 3, 3> C_M_kj = cv::Mat_<double> ( C_Y, cv::Range ( k, k + 3 ), cv::Range ( 3*j, 3*j + 3 ) );
            cv::Mat_<double> tmp = cv::Mat_<double> ( C_MX_k*c_ji[j]->dx.t() + C_M_kj*c_ji[j]->dm.t() );
            K_i_k = tmp * S_inv;
        }

        // I - K_i*H_i
        cv::Mat_<double> I_K_i_H_i = cv::Mat_<double>::eye ( C_Y.rows,  C_Y.cols );
        for ( size_t k = 0; k < C_Y.rows; k += 3 ) {
            // references to submatrices
            cv::Mat_<double> I_K_i_H_i_k0 = cv::Mat_<double> ( I_K_i_H_i, cv::Range ( k, k + 3 ), cv::Range ( 0, 3 ) );
            cv::Mat_<double> I_K_i_H_i_kj = cv::Mat_<double> ( I_K_i_H_i, cv::Range ( k, k + 3 ), cv::Range ( 3*j, 3*j + 3 ) );

            // calculation & matrix update
            cv::Matx<double, 3, 3> K_i_k = cv::Mat_<double> ( K_i, cv::Range ( k, k + 3 ), cv::Range ( 0, 3 ) );
            I_K_i_H_i_k0 -= cv::Mat_<double> ( K_i_k * c_ji[j]->dx );
            I_K_i_H_i_kj -= cv::Mat_<double> ( K_i_k * c_ji[j]->dm );
        }

        // update mean and covariance
        y = y + K_i*v;
        C_Y = I_K_i_H_i*C_Y;

        // normalize angles
        for ( size_t i = 2; i < y.rows; i += 3 )
            y[i][0] = angle_normalize ( y[i][0] );

        // re-establish diagonalization
        C_Y = 0.5 * ( C_Y + C_Y.t() );
    }
}

void EKFSLAM::update_combined() {
    if ( z_known.size() == 0 ) return;

    // initialization
    cv::Mat_<double> v = cv::Mat_<double> ( 3*z_known.size(), 1, 0.0 );
    cv::Mat_<double> H = cv::Mat_<double> ( 3*z_known.size(), C_Y.cols, 0.0 );
    cv::Mat_<double> R = cv::Mat_<double> ( 3*z_known.size(), 3*z_known.size(), 0.0 );
    size_t k = 0;

    for ( auto i: z_known ) {
        assert ( i < c_ij.size() );

        // obtained measurement i corresponds to landmark j
        int j = c_ij[i]->ij.second;
        assert ( 0 < j && j < y.rows/3 && c_ji[j] != nullptr );

        // references to submatrices
        cv::Mat_<double> v_k = cv::Mat_<double> ( v, cv::Range ( k, k + 3 ), cv::Range ( 0, 1 ) );
        cv::Mat_<double> Hx_kj = cv::Mat_<double> ( H, cv::Range ( k, k + 3 ), cv::Range ( 0, 3 ) );
        cv::Mat_<double> Hm_kj = cv::Mat_<double> ( H, cv::Range ( k, k + 3 ), cv::Range ( 3*j, 3*j + 3 ) );
        cv::Mat_<double> R_k = cv::Mat_<double> ( R, cv::Range ( k, k + 3 ), cv::Range ( k, k + 3 ) );

        // calculation & update
        // H = (H_1j H_2j' ... H_Nj'')^T
        // H_kj = (dx_kj 0 .. 0 dm_kj 0 .. 0)
        v_k += cv::Mat_<double> ( c_ji[j]->v );
        Hx_kj += cv::Mat_<double> ( c_ji[j]->dx );
        Hm_kj += cv::Mat_<double> ( c_ji[j]->dm );
        R_k += cv::Mat_<double> ( Q );
        k += 3;
    }

    // calculation
    cv::Mat_<double> S = R + H*C_Y*H.t();
    cv::Mat_<double> K = C_Y*H.t() *S.inv();

    // update mean and covariance
    y = y + K*v;
    C_Y = ( cv::Mat_<double>::eye ( C_Y.rows, C_Y.cols ) - K*H ) *C_Y;

    // normalize angles
    for ( size_t i = 2; i < y.rows; i += 3 )
        y[i][0] = angle_normalize ( y[i][0] );

    // re-establish diagonalization
    C_Y = 0.5 * ( C_Y + C_Y.t() );
}

void EKFSLAM::integration ( const MeasurementFiducialConstPtr &zt ) {
    if ( !config_.enable_integration || z_new.size() == 0 ) return;

    for ( auto i: z_new ) {
        // consider only measurements with fiducial IDs for new landmarks
        assert ( zt->operator[] ( i ).id > 0 );

        // number landmarks found in ascending order
        int j = y.rows/3;
        f_kj.insert ( std::pair<size_t,size_t> ( zt->operator[] ( i ).id, j ) );
        ROS_INFO ( "new landmark found: fiducial %d corresponds to landmark %d", zt->operator[] ( i ).id, j );

        // pre-calculate needed data
        double r = zt->operator[] ( i ).length;
        double T_s = sin ( x[2][0] );
        double T_c = cos ( x[2][0] );
        double F_s = sin ( zt->operator[] ( i ).angle );
        double F_c = cos ( zt->operator[] ( i ).angle );

        // transformation matrix from robot coordinates in global coordinates
        cv::Matx<double, 4, 4> T_x = Pose2D ( x[0][0], x[1][0], x[2][0] ).tf2();

        // transformation matrix from sensor coordinates in robot coordinates
        cv::Matx<double, 4, 4> T_z = zt->getSensorPose().tf2();

        // transformation matrix from sensor coordinates in global coordinates
        cv::Matx<double, 4, 4> T_x_T_z = T_x * T_z;

        // derivation of T_x by theta
        cv::Matx<double, 4, 4> T_x_theta = cv::Matx<double, 4, 4> ( -T_s, -T_c, 0, 0,
                                           T_c, -T_s, 0, 0,
                                           0, 0, 0, 1,
                                           0, 0, 0, 0 );

        // adapt size of global mean
        y.resize ( y.rows + 3 );

        // update gloabl mean with mean of landmark j: m_j = g(x, z) = T_x(x) * T_z * f(z)
        // with f(z) = (r*cos(a), r*sin(a), o), r..radius, a..alpha, o..orientation
        // (function f transforms measurements from spheric coordinates into cartesian coordinates)
        cv::Vec<double, 4> m_j = T_x_T_z * zt->operator[] ( i ).pose.hv();
        y[3*j + 0][0] = m_j[0];
        y[3*j + 1][0] = m_j[1];
        y[3*j + 2][0] = m_j[2];

        // jacobian matrix of f
        cv::Matx<double, 4, 3> F_z = cv::Matx<double, 4, 3> ( F_c, -r*F_s, 0,
                                     F_s,  r*F_c, 0,
                                     0,      0, 1,
                                     0,      0, 0 );

        // jacobian matrix of G in respect to x
        cv::Mat_<double> G_x = cv::Mat_<double>::eye ( 3, 3 );
        cv::Vec<double, 4> G_theta = T_x_theta * T_z * zt->operator[] ( i ).pose.hv();
        G_x[0][2] = G_theta[0];
        G_x[1][2] = G_theta[1];

        // jacobian matrix of G in respect to z
        cv::Matx<double, 3, 3> G_z = cv::Mat_<double> ( cv::Mat_<double> ( T_x_T_z * F_z ), cv::Range ( 0, 3 ), cv::Range ( 0, 3 ) );

        // create horizontal covariance of landmark j
        cv::Mat_<double> C_M_j = cv::Mat_<double> ( 3, C_Y.cols, 0.0 );
        for ( size_t k = 0; k < C_M_j.cols; k += 3 ) {
            // references to submatrices
            cv::Mat_<double> C_M_jk = cv::Mat_<double> ( C_M_j, cv::Range ( 0, 3 ), cv::Range ( k, k + 3 ) );
            cv::Mat_<double> C_XM_k = cv::Mat_<double> ( C_Y, cv::Range ( 0, 3 ), cv::Range ( k, k + 3 ) );

            // calculation & matrix update
            C_M_jk = G_x*C_XM_k;
        }

        // append horizontal covariance of landmark j to global covariance
        cv::Mat tmp_v[] = { C_Y, C_M_j };
        cv::vconcat ( tmp_v, 2, C_Y );

        // create vertical covariance of landmark j
        C_M_j = C_M_j.t();
        C_M_j.resize ( C_M_j.rows + 3 );

        // reference to submatrix
        cv::Mat_<double> C_M_jj = cv::Mat_<double> ( C_M_j, cv::Range ( 3*j, 3*j + 3 ), cv::Range ( 0, 3 ) );

        // calculation & matrix update
        cv::Mat_<double> R = cv::Mat_<double> ( G_z*Q*G_z.t() );
        C_M_jj = G_x*C_X*G_x.t() + R;

        // append vertical covariance of landmark j to global covariance
        cv::Mat tmp_h[] = { C_Y, C_M_j };
        cv::hconcat ( tmp_h, 2, C_Y );

        // reinitialize covariance submatrices pointing to the same data (no copy!)
        x = cv::Mat_<double> ( y, cv::Range ( 0, 3 ), cv::Range ( 0, 1 ) );
        C_X = cv::Mat_<double> ( C_Y, cv::Range ( 0, 3 ), cv::Range ( 0, 3 ) );
    }
}
