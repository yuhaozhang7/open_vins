/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <vector>
#include <sstream>
#include <string>
#include <cstring>

#include <SLAMBenchAPI.h>
#include <io/SLAMFrame.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/CameraSensorFinder.h>
#include <io/sensor/IMUSensor.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ov_msckf/src/core/VioManagerOptions.h"
#include "ov_msckf/src/core/VioManager.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include "config_params.h"
//access to slam objects
static cv::Mat pose;
static cv::Mat frameCV;
static sb_uint2 inputSize;

cv::Mat* img_one;
cv::Mat* img_two;

enum input_mode {mono,stereo};

static input_mode input_m;

static slambench::outputs::Output *pose_output;
static slambench::outputs::Output *frame1_output;
static slambench::outputs::Output *frame2_output;

static slambench::TimeStamp last_frame_timestamp;
static double time_imu, time_cam;
Eigen::Matrix<double, 3, 1> gyr_data, acc_data;
// ===========================================================
// SLAMBench Sensors
// ===========================================================
static slambench::io::CameraSensor *grey_sensor_one = nullptr;
static slambench::io::CameraSensor *grey_sensor_two = nullptr;
static slambench::io::IMUSensor *IMU_sensor = nullptr;

static ov_msckf::VioManagerOptions options;
static ov_msckf::VioManager *sys;

bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings) {

    // State options
    slam_settings->addParameter(TypedParameter<bool>("use_fej", "use_fej",     "fej",    &use_fej, &default_use_fej));
    slam_settings->addParameter(TypedParameter<bool>("use_imu_avg", "use_imu_avg",     "Use IMU averaging",    &use_imu_avg, &default_use_imu_avg));
    slam_settings->addParameter(TypedParameter<bool>("use_rk4int", "use_rk4inttegration",     "",    &use_rk4_integration, &default_use_rk4_integration));
    slam_settings->addParameter(TypedParameter<bool>("use_stereo", "use_stereo",     "",    &use_stereo, &default_use_stereo));
    slam_settings->addParameter(TypedParameter<bool>("calib_cam_extrinsics", "calib_cam_extrinsics",     "",    &do_calib_camera_pose, &default_do_calib_camera_pose));
    slam_settings->addParameter(TypedParameter<bool>("calib_cam_intrinsics", "calib_cam_intrinsics",     "",    &do_calib_camera_intrinsics, &default_do_calib_camera_intrinsics));
    slam_settings->addParameter(TypedParameter<bool>("calib_cam_timeoffset", "calib_cam_timeoffset",    "",    &do_calib_camera_timeoffset, &default_do_calib_camera_timeoffset));
    slam_settings->addParameter(TypedParameter<int>("max_clones", "max_clones",     "",    &max_clones, &default_max_clones));
    slam_settings->addParameter(TypedParameter<int>("max_slam", "max_slam",     "",    &max_slam, &default_max_slam));
    slam_settings->addParameter(TypedParameter<int>("max_aruco", "max_aruco",     "",    &max_aruco, &default_max_aruco));
    slam_settings->addParameter(TypedParameter<int>("max_cameras", "max_cameras",     "",    &max_cameras, &default_num_cameras));
    slam_settings->addParameter(TypedParameter<double>("dt_slam_delay", "dt_slam_delay",     "",    &dt_slam_delay, &default_dt_slam_delay));
    slam_settings->addParameter(TypedParameter<double>("calib_camimu_dt", "calib_camimu_dt",     "",    &calib_camimu_dt, &default_calib_camimu_dt));
    slam_settings->addParameter(TypedParameter<std::string>("fr", "feat_representation",     "",    &feat_representation, &default_feat_representation));

    // Feature initializer options
    slam_settings->addParameter(TypedParameter<int>("fi_max_runs", "fi_max_runs",     "fi_max_runs",    &fi_max_runs, &default_fi_max_runs));
    slam_settings->addParameter(TypedParameter<double>("dt_slam_delay", "dt_slam_delay",     "",    &fi_init_lamda, &default_fi_init_lamda));
    slam_settings->addParameter(TypedParameter<double>("fi_min_dx", "fi_min_dx",     "",    &fi_min_dx, &default_fi_min_dx));
    slam_settings->addParameter(TypedParameter<double>("fi_max_lamda", "fi_max_lamda",     "",    &fi_max_lamda, &default_fi_max_lamda));
    slam_settings->addParameter(TypedParameter<double>("fi_min_dcost", "fi_min_dcost",     "",    &fi_min_dcost, &default_fi_min_dcost));
    slam_settings->addParameter(TypedParameter<double>("fi_lam_mult", "fi_lam_mult",     "",    &fi_lam_mult, &default_fi_lam_mult));
    slam_settings->addParameter(TypedParameter<double>("fi_min_dist", "fi_min_dist",     "",    &fi_min_dist, &default_fi_min_dist));
    slam_settings->addParameter(TypedParameter<double>("fi_max_dist", "fi_max_dist",     "",    &fi_max_dist, &default_fi_max_dist));
    slam_settings->addParameter(TypedParameter<double>("fi_max_baseline", "fi_max_baseline",     "",    &fi_max_baseline, &default_fi_max_baseline));
    slam_settings->addParameter(TypedParameter<double>("fi_max_cond_number", "fi_max_cond_number",     "",    &fi_max_cond_number, &default_fi_max_cond_number));

    slam_settings->addParameter(TypedParameter<double>("init_window_time", "init_window_time",     "",    &init_window_time, &default_init_window_time));
    slam_settings->addParameter(TypedParameter<double>("init_imu_thresh", "init_imu_thresh",     "",    &init_imu_thresh, &default_init_imu_thresh));

    // Extractor options
    slam_settings->addParameter(TypedParameter<bool>("use_klt", "use_klt",     "",    &use_klt, &default_use_klt));
    slam_settings->addParameter(TypedParameter<bool>("use_aruco", "use_aruco",     "",    &use_aruco, &default_use_aruco));
    slam_settings->addParameter(TypedParameter<bool>("ds_aruco", "downsize_aruco",     "",    &downsize_aruco, &default_downsize_aruco));
    slam_settings->addParameter(TypedParameter<int>("fi_max_runs", "fi_max_runs",     "fi_max_runs",    &fi_max_runs, &default_fi_max_runs));
    slam_settings->addParameter(TypedParameter<int>("num_pts", "num_pts",     "",    &num_pts, &default_num_pts));
    slam_settings->addParameter(TypedParameter<int>("fast_threshold", "fast_threshold",     "",    &fast_threshold, &default_fast_threshold));
    slam_settings->addParameter(TypedParameter<int>("grid_x", "grid_x",     "",    &grid_x, &default_grid_x));
    slam_settings->addParameter(TypedParameter<int>("grid_y", "grid_y",     "",    &grid_y, &default_grid_y));
    slam_settings->addParameter(TypedParameter<int>("min_px_dist", "min_px_dist",     "",    &min_px_dist, &default_min_px_dist));
    slam_settings->addParameter(TypedParameter<double>("knn_ratio", "knn_ratio",     "",    &knn_ratio, &default_knn_ratio));


    // Update parameters
    slam_settings->addParameter(TypedParameter<double>("up_msckf_sigma_px", "up_msckf_sigma_px",     "",    &up_msckf_sigma_px, &default_up_msckf_sigma_px));
    slam_settings->addParameter(TypedParameter<double>("up_slam_sigma_px", "up_slam_sigma_px",     "",    &up_slam_sigma_px, &default_up_slam_sigma_px));
    slam_settings->addParameter(TypedParameter<double>("up_aruco_sigma_px", "up_aruco_sigma_px",     "",    &up_aruco_sigma_px, &default_up_aruco_sigma_px));
    slam_settings->addParameter(TypedParameter<int>("up_msckf_chi2_multipler", "up_msckf_chi2_multipler",     "",    &up_msckf_chi2_multipler, &default_up_msckf_chi2_multipler));
    slam_settings->addParameter(TypedParameter<int>("up_slam_chi2_multipler", "up_slam_chi2_multipler",     "",    &up_slam_chi2_multipler, &default_up_slam_chi2_multipler));
    slam_settings->addParameter(TypedParameter<int>("up_aruco_chi2_multipler", "up_aruco_chi2_multipler",     "",    &up_aruco_chi2_multipler, &default_up_aruco_chi2_multipler));


    // If our distortions are fisheye or not!
    is_fisheye.reserve(max_cameras);
    for(int i = 0; i < max_cameras; i++)
    {
        auto is_fisheye_name = "cam"+std::to_string(i)+"_is_fisheye";
        auto name = "cam"+std::to_string(i)+"_k";
        bool temp_is_fisheye;

        slam_settings->addParameter(TypedParameter<bool>(name, name, name, &temp_is_fisheye, &default_is_fisheye));

//        nh.param<std::vector<double>>(, , matrix_k_default);
//        nh.param<std::vector<double>>("cam"+std::to_string(i)+"_d", matrix_d, matrix_d_default);
//        nh.param<std::vector<double>>("T_C"+std::to_string(i)+"toI", matrix_TCtoI, matrix_TtoI_default);
//        nh.param<std::vector<int>>("cam"+std::to_string(i)+"_wh", matrix_wh, matrix_wd_default);

        is_fisheye[i] = temp_is_fisheye;
    }


    return true;
}

bool sb_init_slam_system(SLAMBenchLibraryHelper * slam_settings)  {


    //=========================================================================
    // We collect sensors
    //=========================================================================

    slambench::io::CameraSensorFinder sensor_finder;
    IMU_sensor = (slambench::io::IMUSensor*)slam_settings->get_sensors().GetSensor(slambench::io::IMUSensor::kIMUType);
    if(IMU_sensor == nullptr) {
        std::cout << "Init failed, did not found IMU." << std::endl;
        return false;
    }

    auto grey_sensors = sensor_finder.Find(slam_settings->get_sensors(), {{"camera_type", "grey"}});
    grey_sensor_one = grey_sensors.at(0);

    if(grey_sensors.size() ==  2) {
        grey_sensor_two = grey_sensors.at(1);
    }

    assert(grey_sensor_one);
    if (grey_sensor_two)
        input_m = input_mode::stereo;
    else
        input_m = input_mode::mono;

    if  ( input_m == input_mode::mono ) {
        //=========================================================================
        // MONOCULAR Mode
        //=========================================================================
        cv::Mat camera_parameters = cv::Mat::eye(3,3,CV_32F);
        cv::Mat camera_distortion(5,1,CV_32F);

        camera_parameters.at<float>(0,0) = grey_sensor_one->Intrinsics[0]*grey_sensor_one->Width;
        camera_parameters.at<float>(1,1) = grey_sensor_one->Intrinsics[1]*grey_sensor_one->Height;
        camera_parameters.at<float>(0,2) = grey_sensor_one->Intrinsics[2]*grey_sensor_one->Width;
        camera_parameters.at<float>(1,2) = grey_sensor_one->Intrinsics[3]*grey_sensor_one->Height;

        img_one     = new cv::Mat ( grey_sensor_one->Height ,  grey_sensor_one->Width, CV_8UC1);
        inputSize   = make_sb_uint2(grey_sensor_one->Width,grey_sensor_one->Height);
    }
    else if ( input_m == input_mode::stereo )  {


        //=========================================================================
        // STEREO Mode
        //=========================================================================
        assert(grey_sensor_two);

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;

        // Intrisics
        K_l = cv::Mat::zeros(3, 3,CV_64F);
        K_l.at<double>(0,0) =  grey_sensor_one->Intrinsics[0]*grey_sensor_one->Width;
        K_l.at<double>(1,1) =  grey_sensor_one->Intrinsics[1]*grey_sensor_one->Height;
        K_l.at<double>(0,2) =  grey_sensor_one->Intrinsics[2]*grey_sensor_one->Width;
        K_l.at<double>(1,2) =  grey_sensor_one->Intrinsics[3]*grey_sensor_one->Height;
        K_l.at<double>(2,2) =  1.0;

        K_r = cv::Mat::zeros(3, 3,CV_64F);
        K_r.at<double>(0,0) =  grey_sensor_two->Intrinsics[0]*grey_sensor_two->Width;
        K_r.at<double>(1,1) =  grey_sensor_two->Intrinsics[1]*grey_sensor_two->Height;
        K_r.at<double>(0,2) =  grey_sensor_two->Intrinsics[2]*grey_sensor_two->Width;
        K_r.at<double>(1,2) =  grey_sensor_two->Intrinsics[3]*grey_sensor_two->Height;
        K_r.at<double>(2,2) =  1.0;

        // Distortion
        D_l = cv::Mat::zeros(1, 5,CV_64F);
        D_l.at<double>(0,0) =  grey_sensor_one->RadialTangentialDistortion[0];
        D_l.at<double>(0,1) =  grey_sensor_one->RadialTangentialDistortion[1];
        D_l.at<double>(0,2) =  grey_sensor_one->RadialTangentialDistortion[2];
        D_l.at<double>(0,3) =  grey_sensor_one->RadialTangentialDistortion[3];
        D_l.at<double>(0,4) =  grey_sensor_one->RadialTangentialDistortion[4];

        D_r = cv::Mat::zeros(1, 5,CV_64F);
        D_r.at<double>(0,0) =  grey_sensor_two->RadialTangentialDistortion[0];
        D_r.at<double>(0,1) =  grey_sensor_two->RadialTangentialDistortion[1];
        D_r.at<double>(0,2) =  grey_sensor_two->RadialTangentialDistortion[2];
        D_r.at<double>(0,3) =  grey_sensor_two->RadialTangentialDistortion[3];
        D_r.at<double>(0,4) =  grey_sensor_two->RadialTangentialDistortion[4];



        // Height and width

        int rows_l = grey_sensor_one->Height;
        int cols_l = grey_sensor_one->Width;
        int rows_r = grey_sensor_two->Height;
        int cols_r = grey_sensor_two->Width;



        std::vector<cv::Mat> vK, vD, vTBS;
        std::vector<cv::Size> vSz;

        // //////////////////////////////////////////////////////////

        // here we read T_BS, K , D and images size for each camera

        // read TBS

        cv::Mat T_BS_l(4,4, CV_64F);

        for (int r = 0; r < 4; r++)

            for (int c = 0; c < 4; c++)

                T_BS_l.at<double>(r,c) =  grey_sensor_one->Pose(r,c) ;

        cv::Mat T_BS_r(4,4, CV_64F);

        for (int r = 0; r < 4; r++)

            for (int c = 0; c < 4; c++)

                T_BS_r.at<double>(r,c) =  grey_sensor_two->Pose(r,c) ;


        // //////////////////////////////////////////////////////////

        cv::Mat R1,R2,P1,P2,Q;

        cv::Mat Tr = (T_BS_r).inv() * (T_BS_l);

        cv::Mat R, T;

        Tr.colRange(0,3).rowRange(0,3).copyTo(R);

        Tr.col(3).rowRange(0,3).copyTo(T);



        // note that order of cameras matter (left camera, ca,era 1) sould be the first one.

//        cv::stereoRectify(K_l, D_l, K_r, D_r, cv::Size(cols_l,rows_l), R, T, R_l, R_r, P_l, P_r, Q, CV_CALIB_ZERO_DISPARITY,0);

        double bf = std::abs (	P_r.at<double>(0,3)  - 	P_l.at<double>(0,3) ) ;

        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = P_l.at<double>(0,0);
        K.at<float>(1,1) = P_l.at<double>(1,1);
        K.at<float>(0,2) = P_l.at<double>(0,2);
        K.at<float>(1,2) = P_l.at<double>(1,2);


        cv::Mat DistCoef(4,1,CV_32F);
        DistCoef.at<float>(0) = 0.0;
        DistCoef.at<float>(1) = 0.0;
        DistCoef.at<float>(2) = 0.0;
        DistCoef.at<float>(3) = 0.0;

        img_one = new cv::Mat ( grey_sensor_one->Height ,  grey_sensor_one->Width, CV_8UC1);
        img_two = new cv::Mat ( grey_sensor_two->Height ,  grey_sensor_two->Width, CV_8UC1);
        inputSize   = make_sb_uint2(grey_sensor_one->Width,grey_sensor_one->Height);

    }
    else {

        std::cout << "Invalid input mode '" << input_m << "'" << std::endl;
        exit(1);
    }
//    parameters.imu.sigma_g_c  = ;
//    parameters.imu.sigma_gw_c = IMU_sensor->GyroscopeDriftNoiseDensity;
//    parameters.imu.sigma_bg  = IMU_sensor->GyroscopeBiasDiffusion;
//    parameters.imu.g_max  = IMU_sensor->GyroscopeSaturation;
//
//    parameters.imu.sigma_a_c  = ;
//    parameters.imu.sigma_aw_c  = IMU_sensor->AcceleratorDriftNoiseDensity;
//    parameters.imu.sigma_ba  = IMU_sensor->AcceleratorBiasDiffusion;
//    parameters.imu.a_max  =IMU_sensor->AcceleratorSaturation;
    // Inertial sensor noise values and initial parameters
    gyroscope_noise_density = IMU_sensor->GyroscopeNoiseDensity;
    accelerometer_noise_density = IMU_sensor->AcceleratorNoiseDensity;


    slam_settings->addParameter(TypedParameter<double>("gyroscope_random_walk", "gyroscope_random_walk",     "",    &gyroscope_random_walk, &default_gyroscope_random_walk));
    slam_settings->addParameter(TypedParameter<double>("accelerometer_random_walk", "accelerometer_random_walk",     "",    &accelerometer_random_walk, &default_accelerometer_random_walk));


    options.cam_calib << matrix_k.at(0),matrix_k.at(1),matrix_k.at(2),matrix_k.at(3),matrix_d.at(0),matrix_d.at(1),matrix_d.at(2),matrix_d.at(3);
    options.wh = std::pair<int,int>(matrix_wh.at(0),matrix_wh.at(1));
    options.T_CtoI << matrix_TCtoI.at(0),matrix_TCtoI.at(1),matrix_TCtoI.at(2),matrix_TCtoI.at(3),
            matrix_TCtoI.at(4),matrix_TCtoI.at(5),matrix_TCtoI.at(6),matrix_TCtoI.at(7),
            matrix_TCtoI.at(8),matrix_TCtoI.at(9),matrix_TCtoI.at(10),matrix_TCtoI.at(11),
            matrix_TCtoI.at(12),matrix_TCtoI.at(13),matrix_TCtoI.at(14),matrix_TCtoI.at(15);
    options.calib_camimu_dt(0) = calib_camimu_dt;


    sys = new ov_msckf::VioManager(options);



    pose_output = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);
    slam_settings->GetOutputManager().RegisterOutput(pose_output);

    frame1_output = new slambench::outputs::Output("Input Frame", slambench::values::VT_FRAME);
    frame1_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(frame1_output);

    frame2_output = new slambench::outputs::Output("Tracking frame", slambench::values::VT_FRAME);
    frame2_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(frame2_output);


    return true;

}

bool grey_one_ready = false, grey_two_ready = false;
bool sb_update_frame (SLAMBenchLibraryHelper * , slambench::io::SLAMFrame* s) {
    assert(s != nullptr);

    if(s->FrameSensor == IMU_sensor) {

        float* frame_data = (float*)s->GetData();
        time_imu = s->Timestamp.S;
        gyr_data << frame_data[0], frame_data[1], frame_data[2];
        acc_data << frame_data[3], frame_data[4], frame_data[5];
    }
    else if(s->FrameSensor == grey_sensor_one and img_one) {
        memcpy(img_one->data, s->GetData(), s->GetSize());
        grey_one_ready = true;
        s->FreeData();
    }
    else if(s->FrameSensor == grey_sensor_two and img_two) {
        memcpy(img_two->data, s->GetData(), s->GetSize());
        grey_two_ready = true;
        time_cam = s->Timestamp.ToS();
        s->FreeData();
    }
    last_frame_timestamp = s->Timestamp;

    return (input_m == input_mode::mono and grey_one_ready) or
           (input_m == input_mode::stereo and grey_one_ready and grey_two_ready);
}

bool sb_process_once (SLAMBenchLibraryHelper * slam_settings)  {

    // send time, angular velocity, linear acceleration
    sys->feed_measurement_imu(time_imu, gyr_data, acc_data);

    if (input_m == input_mode::mono) {
        sys->feed_measurement_monocular(time_cam, *img_one, 0);
        grey_one_ready = false;
    }
    else if (input_m == input_mode::stereo && grey_one_ready and grey_two_ready) {
        // process once we have initialized with the GT
        Eigen::Matrix<double, 17, 1> imustate;
        if(!gt_states.empty() && !sys->intialized()) {
            //biases are pretty bad normally, so zero them
            //imustate.block(11,0,6,1).setZero();
            sys->initialize_with_gt(imustate);
        } else if(gt_states.empty() || sys->intialized()) {
            sys->feed_measurement_stereo(time_cam, *img_one, *img_two, 0, 1);
        }

        grey_one_ready = false;
        grey_two_ready = false;

    }
    else {
        std::cout << "Unsupported case." << std::endl;
    }

    return true;
}


bool sb_clean_slam_system() {
    delete sys;
    return true;
}

bool sb_get_pose (Eigen::Matrix4f * mat)  {
    for(int j=0; j<4;j++) {
        for(int i=0; i<4;i++) {
            (*mat)(j,i)= pose.at<float>(j,i);
        }
    }
    return true;
}

bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *latest_output) {
    (void)lib;

    if(pose_output->IsActive()) {
        // Get the current pose as an eigen matrix
        Eigen::Matrix4f matrix;
        sb_get_pose(&matrix);

        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        pose_output->AddPoint(last_frame_timestamp, new slambench::values::PoseValue(matrix));
    }

    if(frame1_output->IsActive() && img_one) {

        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
            frame1_output->AddPoint(last_frame_timestamp, new slambench::values::FrameValue(inputSize.x, inputSize.y,
                                                                                            slambench::io::pixelformat::G_I_8,
                                                                                            (void *) (img_one->data)));

    }
//    TODO: show 2nd image here
    if(frame2_output->IsActive() ) {
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        frame2_output->AddPoint(last_frame_timestamp, new slambench::values::FrameValue(inputSize.x, inputSize.y, slambench::io::pixelformat::RGB_III_888,  (void*)(&frameCV.at<char>(0,0))));
    }



    return true;
}




