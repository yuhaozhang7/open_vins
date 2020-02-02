#ifndef OPEN_VINS_VIOMANAGEROPTIONS_H
#define OPEN_VINS_VIOMANAGEROPTIONS_H
#include "../state/StateOptions.h"
#include "../../../ov_core/src/feat/FeatureInitializerOptions.h"
#include "../state/Propagator.h"
#include "../update/UpdaterOptions.h"

namespace ov_msckf {
    struct VioManagerOptions
    {
        StateOptions state_options;
        Eigen::Matrix<double,3,1> gravity;
        Eigen::Matrix<double,1,1> calib_camimu_dt;
        FeatureInitializerOptions featinit_options;
        Propagator::NoiseManager imu_noises;
        std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
        // Parameters for our extractor
        int num_pts, fast_threshold, grid_x, grid_y, min_px_dist;
        double knn_ratio;
        bool use_klt, use_aruco, do_downsizing;
        double init_window_time, init_imu_thresh;

        bool is_fisheye;
        std::pair<int,int> wh;
        UpdaterOptions msckf_options, slam_options, aruco_options;
        Eigen::Matrix4d T_CtoI;
        std::string feat_rep_str;
        Eigen::Matrix<double,8,1> cam_calib;
    };
}
#endif //OPEN_VINS_VIOMANAGEROPTIONS_H
