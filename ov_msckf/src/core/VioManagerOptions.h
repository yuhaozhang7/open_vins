#ifndef OPEN_VINS_VIOMANAGEROPTIONS_H
#define OPEN_VINS_VIOMANAGEROPTIONS_H
namespace ov_msckf {
    struct VioManagerOptions
    {
        StateOptions& state_options;
        Eigen::Matrix<double,3,1> gravity;
        Eigen::Matrix<double,1,1> calib_camimu_dt;
        FeatureInitializerOptions featinit_options;
        Propagator::NoiseManager imu_noises;

        // Parameters for our extractor
        int num_pts, fast_threshold, grid_x, grid_y, min_px_dist;
        double knn_ratio;
        bool use_klt, use_aruco, do_downsizing;
        double init_window_time, init_imu_thresh;
        bool is_fisheye;
        std::pair<int,int> wh;
        UpdaterOptions msckf_options, slam_options, aruco_options;
        Eigen::Matrix4d T_CtoI;
    };
}
#endif //OPEN_VINS_VIOMANAGEROPTIONS_H
