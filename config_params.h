#ifndef SLAMBENCH2_REPOSITORY_CONFIG_PARAMS_H
#define SLAMBENCH2_REPOSITORY_CONFIG_PARAMS_H

// ===========================================================
// State options
// ===========================================================
static bool use_fej,
        use_imu_avg,
        use_rk4_integration,
        use_stereo,
        do_calib_camera_pose,
        do_calib_camera_intrinsics,
        do_calib_camera_timeoffset;

static int max_clones,
        max_slam,
        max_aruco,
        max_cameras;

static double dt_slam_delay, calib_camimu_dt;
static const double default_dt_slam_delay = 3, default_calib_camimu_dt = 0.0;

static std::string feat_representation;
static const std::string default_feat_representation = "GLOBAL_3D";

static const bool default_use_fej = false,
        default_use_imu_avg = false,
        default_use_rk4_integration = true,
        default_use_stereo = true,
        default_do_calib_camera_pose = false,
        default_do_calib_camera_intrinsics = false,
        default_do_calib_camera_timeoffset = false;

static const int default_max_clones = 10,
        default_max_slam = 0,
        default_max_aruco = 1024,
        default_max_cameras = 2;


// ===========================================================
// Feature initializer options
// ===========================================================
static int fi_max_runs;
static const int default_fi_max_runs = 20;

static double fi_init_lamda, fi_max_lamda, fi_min_dx, fi_min_dcost, fi_lam_mult, fi_min_dist,
        fi_max_dist, fi_max_baseline, fi_max_cond_number;

static const double default_fi_init_lamda = 1e-3,
        default_fi_max_lamda = 1e10,
        default_fi_min_dx = 1e-6,
        default_fi_min_dcost = 1e-6,
        default_fi_lam_mult = 10,
        default_fi_min_dist = 0.25,
        default_fi_max_dist = 40,
        default_fi_max_baseline = 40,
        default_fi_max_cond_number = 1000;


// ===========================================================
// Extractor options
// ===========================================================
static bool use_klt, use_aruco, downsize_aruco;
static const bool default_use_klt = true,
        default_use_aruco = false,
        default_downsize_aruco = true;

static int num_pts,
        fast_threshold,
        grid_x, grid_y,
        min_px_dist;

static const int default_num_pts = 500,
        default_fast_threshold = 10,
        default_grid_x = 10,
        default_grid_y = 8,
        default_min_px_dist = 10;

static double knn_ratio;
static const double default_knn_ratio = 0.85;


// ===========================================================
// Inertial sensor noise values and initial parameters
// ===========================================================
static double init_window_time,
              init_imu_thresh;
static const double default_init_window_time = 0.5,
                    default_init_imu_thresh = 1.0;


// ===========================================================
// Update parameters
// ===========================================================
static double up_msckf_sigma_px,
        up_slam_sigma_px,
        up_aruco_sigma_px;
static const double default_up_msckf_sigma_px = 1,
        default_up_slam_sigma_px = 1,
        default_up_aruco_sigma_px = 1;

static int up_msckf_chi2_multipler,
        up_slam_chi2_multipler,
        up_aruco_chi2_multipler;
static const int default_up_msckf_chi2_multipler = 5,
        default_up_slam_chi2_multipler = 5,
        default_up_aruco_chi2_multipler = 5;


// Camera intrinsic properties
std::vector<bool> is_fisheye;
static const bool default_is_fisheye = false;

std::vector<double> matrix_k_default = {458.654,457.296,367.215,248.375};
std::vector<double> matrix_d_default = {-0.28340811,0.07395907,0.00019359,1.76187114e-05};

// If the desired fov we should simulate
std::vector<int> matrix_wh;
std::vector<int> matrix_wd_default = {752,480};

// Our camera extrinsics transform

std::vector<double> matrix_TtoI_default = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
std::vector<double> vec_gravity;
std::vector<double> vec_gravity_default = {0.0,0.0,9.81};

std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
#endif //SLAMBENCH2_REPOSITORY_CONFIG_PARAMS_H
