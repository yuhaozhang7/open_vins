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
        num_cameras;

static double dt_slam_delay, calib_camimu_dt;
static const double default_dt_slam_delay = 5, default_calib_camimu_dt = 0.0;

static std::string feat_representation;
static const std::string default_feat_representation = "GLOBAL_3D";

static const bool default_use_fej = true,
        default_use_imu_avg = true,
        default_use_rk4_integration = true,
        default_use_stereo = true,
        default_do_calib_camera_pose = true,
        default_do_calib_camera_intrinsics = true,
        default_do_calib_camera_timeoffset = true;

static const int default_max_clones = 11,
        default_max_slam = 50,
        default_max_aruco = 1024,
        default_num_cameras = 2;


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

static const int default_num_pts = 400,
        default_fast_threshold = 15,
        default_grid_x = 5,
        default_grid_y = 3,
        default_min_px_dist = 10;

static double knn_ratio;
static const double default_knn_ratio = 0.7;


// ===========================================================
// Inertial sensor noise values and initial parameters
// ===========================================================
static double init_window_time,
              init_imu_thresh;
static const double default_init_window_time = 0.75,
                    default_init_imu_thresh = 1.5;


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
static const int default_up_msckf_chi2_multipler = 1,
        default_up_slam_chi2_multipler = 1,
        default_up_aruco_chi2_multipler = 1;

static int max_slam_in_update, max_msckf_in_update;
static const int default_max_slam_in_update = 25, default_max_msckf_in_update = 999;

// Camera intrinsic properties
std::vector<bool> is_fisheye;
static const bool default_is_fisheye = false;

std::vector<double> vec_gravity;
static const std::vector<double> vec_gravity_default = {0.0,0.0,9.81};

std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
std::string path_gt, default_path_gt = "";
#endif //SLAMBENCH2_REPOSITORY_CONFIG_PARAMS_H
