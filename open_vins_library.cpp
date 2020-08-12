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
#include "ov_core/src/utils/dataset_reader.h"
//access to slam objects
static cv::Mat pose;
static std::vector<sb_uint2> camera_input_s;

std::vector<cv::Mat*> img;
std::vector<cv::Mat*> img_buffers;

static slambench::outputs::Output *pose_output;
static std::vector<slambench::outputs::Output *>frame_outputs;

static slambench::TimeStamp last_frame_timestamp;
static double time_imu, time_cam, time_cam_buffer;
Eigen::Matrix<double, 3, 1> gyr_data, acc_data;
std::vector<bool> grey_ready;

// SLAMBench Sensors
static std::vector<slambench::io::CameraSensor*> grey_sensors;
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
  slam_settings->addParameter(TypedParameter<int>("num_cameras", "num_cameras",     "",    &num_cameras, &default_num_cameras));
  slam_settings->addParameter(TypedParameter<double>("dt_slam_delay", "dt_slam_delay",     "",    &dt_slam_delay, &default_dt_slam_delay));
  slam_settings->addParameter(TypedParameter<double>("calib_camimu_dt", "calib_camimu_dt",     "",    &calib_camimu_dt, &default_calib_camimu_dt));
  slam_settings->addParameter(TypedParameter<std::string>("fr", "feat_representation",     "",    &feat_representation, &default_feat_representation));

  // Feature initializer options
  slam_settings->addParameter(TypedParameter<int>("fi_max_runs", "fi_max_runs",     "fi_max_runs",    &fi_max_runs, &default_fi_max_runs));
  slam_settings->addParameter(TypedParameter<double>("fi_init_lamda", "fi_init_lamda",     "",    &fi_init_lamda, &default_fi_init_lamda));
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
  slam_settings->addParameter(TypedParameter<int>("max_slam_in_update", "max_slam_in_update",     "",    &max_slam_in_update, &default_max_slam_in_update));
  slam_settings->addParameter(TypedParameter<int>("max_msckf_in_update", "max_msckf_in_update",     "",    &max_msckf_in_update, &default_max_msckf_in_update));

  slam_settings->addParameter(TypedParameter<std::string>("path_gt", "path_gt", "Path to ground truth poses", &path_gt, &default_path_gt));

  // If our distortions are fisheye or not!
  for(int i = 0; i < num_cameras; i++)
  {
    auto is_fisheye_name = "cam"+std::to_string(i)+"_is_fisheye";
    bool temp_is_fisheye;
    slam_settings->addParameter(TypedParameter<bool>(is_fisheye_name, is_fisheye_name, is_fisheye_name, &temp_is_fisheye, &default_is_fisheye));
    is_fisheye.push_back(temp_is_fisheye);
  }

    return true;
}

bool sb_init_slam_system(SLAMBenchLibraryHelper * slam_settings)  {
    // Get sensors
    slambench::io::CameraSensorFinder sensor_finder;
    IMU_sensor = (slambench::io::IMUSensor*)slam_settings->get_sensors().GetSensor(slambench::io::IMUSensor::kIMUType);
    assert(IMU_sensor != nullptr && "Init failed, did not found IMU.");

    grey_sensors = sensor_finder.Find(slam_settings->get_sensors(), {{"camera_type", "grey"}});
    assert(grey_sensors[0] && "At least one camera needed");

    for ( auto i = 0; i < num_cameras; i++ )  {
        options.camera_fisheye.insert({i, is_fisheye[i]});
        options.camera_wh.insert({i,std::pair<int,int>(grey_sensors[i]->Width,grey_sensors[i]->Height)});
        Eigen::Matrix<double,8,1> cam_calib;
        cam_calib << grey_sensors[i]->Intrinsics[0]*grey_sensors[i]->Width,
                     grey_sensors[i]->Intrinsics[1]*grey_sensors[i]->Height,
                     grey_sensors[i]->Intrinsics[2]*grey_sensors[i]->Width,
                     grey_sensors[i]->Intrinsics[3]*grey_sensors[i]->Height,
                     grey_sensors[i]->Distortion[0],
                     grey_sensors[i]->Distortion[1],
                     grey_sensors[i]->Distortion[2],
                     grey_sensors[i]->Distortion[3];
        options.camera_intrinsics.insert({i,cam_calib});

        //auto vec_TCtoI = matrix_TCtoI_vec.at(i);
        Eigen::Matrix4d T_CtoI = grey_sensors[i]->Pose.cast<double>();
        //T_CtoI << vec_TCtoI.at(0), vec_TCtoI.at(1), vec_TCtoI.at(2), vec_TCtoI.at(3),
        //    vec_TCtoI.at(4), vec_TCtoI.at(5), vec_TCtoI.at(6), vec_TCtoI.at(7),
        //    vec_TCtoI.at(8), vec_TCtoI.at(9), vec_TCtoI.at(10), vec_TCtoI.at(11),
        //    vec_TCtoI.at(12), vec_TCtoI.at(13), vec_TCtoI.at(14), vec_TCtoI.at(15);
        Eigen::Matrix<double, 4, 1> rot_quat = rot_2_quat(T_CtoI.block(0,0,3,3).transpose());
        Eigen::Matrix<double, 3, 1> tr_quat = T_CtoI.block(0,3,3,1);
        Eigen::Matrix<double, 7, 1> extrinsics;
        extrinsics<< rot_quat(0), rot_quat(1), rot_quat(2), rot_quat(3), tr_quat(0), tr_quat(1), tr_quat(2);
        options.camera_extrinsics.insert({i,extrinsics});

        img.push_back(new cv::Mat(grey_sensors[i]->Height, grey_sensors[i]->Width,CV_8UC1));
        img_buffers.push_back(new cv::Mat());
        camera_input_s.push_back(make_sb_uint2(grey_sensors[i]->Width, grey_sensors[i]->Height));
        grey_ready.push_back(false);

        auto frame_out = new slambench::outputs::Output("Frame_"+to_string(i), slambench::values::VT_FRAME);
        frame_out->SetKeepOnlyMostRecent(true);
        slam_settings->GetOutputManager().RegisterOutput(frame_out);
        frame_outputs.push_back(frame_out);
    }
    // TODO: delay for both cameras when using stereo?
    options.calib_camimu_dt = calib_camimu_dt != 0.0 ? calib_camimu_dt : grey_sensors[0]->Delay;

    options.state_options.feat_rep_slam = ov_type::LandmarkRepresentation::from_string(feat_representation);
    options.state_options.feat_rep_aruco = options.state_options.feat_rep_slam;
    options.state_options.do_fej = use_fej;
    options.state_options.imu_avg = use_imu_avg;
    options.state_options.use_rk4_integration = use_rk4_integration;
    options.state_options.do_calib_camera_pose = do_calib_camera_pose;
    options.state_options.do_calib_camera_intrinsics = do_calib_camera_intrinsics;
    options.state_options.do_calib_camera_timeoffset = do_calib_camera_timeoffset;
    options.state_options.max_clone_size = max_clones;
    options.state_options.max_slam_features = max_slam;
    options.state_options.max_aruco_features = max_aruco;
    options.state_options.num_cameras = num_cameras;
    options.state_options.max_slam_in_update = max_slam_in_update;
    options.state_options.max_msckf_in_update = max_msckf_in_update;
    options.dt_slam_delay = dt_slam_delay;

    options.use_klt = use_klt;
    options.use_aruco = use_aruco;
    options.num_pts = num_pts;
    options.fast_threshold = fast_threshold;
    options.grid_x = grid_x;
    options.grid_y = grid_y;
    options.min_px_dist = min_px_dist;
    options.knn_ratio = knn_ratio;
    options.downsize_aruco = downsize_aruco;
    options.min_px_dist = min_px_dist;

    options.featinit_options.max_runs = fi_max_runs;
    options.featinit_options.init_lamda = fi_init_lamda;
    options.featinit_options.max_lamda = fi_max_lamda;
    options.featinit_options.min_dx = fi_min_dx;
    options.featinit_options.min_dcost = fi_min_dcost;
    options.featinit_options.lam_mult = fi_lam_mult;
    options.featinit_options.min_dist = fi_min_dist;
    options.featinit_options.max_dist = fi_max_dist;
    options.featinit_options.max_baseline = fi_max_baseline;
    options.featinit_options.max_cond_number = fi_max_cond_number;
    //    parameters.imu.g_max  = IMU_sensor->GyroscopeSaturation;
    //    parameters.imu.a_max  = IMU_sensor->AcceleratorSaturation;
    // Inertial sensor noise values and initial parameters
    options.imu_noises.sigma_w = IMU_sensor->GyroscopeNoiseDensity;
    options.imu_noises.sigma_a = IMU_sensor->AcceleratorNoiseDensity;
    options.imu_noises.sigma_wb  = IMU_sensor->GyroscopeBiasDiffusion;//gyroscope_random_walk
    options.imu_noises.sigma_ab = IMU_sensor->AcceleratorBiasDiffusion;//accelerometer_random_walk
    options.init_window_time = init_window_time;
    options.init_imu_thresh = init_imu_thresh;
    options.gravity << vec_gravity_default[0],vec_gravity_default[1],vec_gravity_default[2];

    sys = new ov_msckf::VioManager(options);

    if (!path_gt.empty()) {
        DatasetReader::load_gt_file(path_gt, gt_states);
    }

    pose_output = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);
    slam_settings->GetOutputManager().RegisterOutput(pose_output);

    return true;

}

bool sb_update_frame (SLAMBenchLibraryHelper * , slambench::io::SLAMFrame* s) {
    assert(s != nullptr);

    if(s->FrameSensor == IMU_sensor) {
        float* frame_data = (float*)s->GetData();
        time_imu = s->Timestamp.ToS();
        gyr_data << frame_data[0], frame_data[1], frame_data[2];
        acc_data << frame_data[3], frame_data[4], frame_data[5];
        // IMU data needs to be sent as soon as it arrives, don't wait for greyscale frames
        sys->feed_measurement_imu(time_imu, gyr_data, acc_data);
    }
    else
    {
        for(size_t i = 0; i < grey_sensors.size(); i++)
        {
            if(s->FrameSensor == grey_sensors[i]) {
                memcpy(img[i]->data, s->GetData(), s->GetSize());
                time_cam = s->Timestamp.ToS();
                if(img_buffers[i]->rows == 0)
                {
                    time_cam_buffer = time_cam;
                    *img_buffers[i] = img[i]->clone();
                }
                grey_ready[i] = true;
                s->FreeData();
                break;
            }
        }
    }

    last_frame_timestamp = s->Timestamp;
    for(bool ready : grey_ready)
        if(!ready)
            return false;
    return true;
}

bool sb_process_once (SLAMBenchLibraryHelper * slam_settings)  {

    // monocular
    if (grey_ready.size() == 1 and grey_ready[0]) {
        sys->feed_measurement_monocular(time_cam_buffer, *img_buffers[0], 0);
        grey_ready[0] = false;
        *img_buffers[0] = img[0]->clone();
    }
    else if (grey_ready.size() == 2 and grey_ready[0] and grey_ready[1]) {
        // process once we have initialized with the GT
        Eigen::Matrix<double, 17, 1> imustate;
        if(!gt_states.empty() and !sys->initialized() and DatasetReader::get_gt_state(time_cam_buffer,imustate,gt_states)) {
            //biases are pretty bad normally, so zero them
            //imustate.block(11,0,6,1).setZero();
            sys->initialize_with_gt(imustate);
        }
        if(gt_states.empty() || sys->initialized()) {
            sys->feed_measurement_stereo(time_cam_buffer, *img_buffers[0], *img_buffers[1], 0, 1);
        }
        for(int i = 0; i < grey_ready.size();i++)
        {
            *img_buffers[i] = img[i]->clone();
            grey_ready[i] = false;
        }
    }
    time_cam_buffer = time_cam;
    return true;
}

bool sb_clean_slam_system() {
    delete sys;
    delete pose_output;
    for(auto sensor : grey_sensors)
        delete sensor;
    delete IMU_sensor;
    for(auto output : frame_outputs)
        delete output;
    return true;
}

bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *latest_output) {
    (void)lib;

    // Send pose
    if(pose_output->IsActive()) {
        auto imu_rot = sys->get_state()->imu()->Rot().transpose();
        auto imu_pos = sys->get_state()->imu()->pos();

        // Get the current pose as an eigen matrix
        Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
        matrix.block<3,3>(0,0) = imu_rot.cast<float>();
        matrix.block<3,1>(0,3) = imu_pos.cast<float>();

        std::cout<< "pose:" << matrix << std::endl;
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        pose_output->AddPoint(*latest_output, new slambench::values::PoseValue(matrix));
    }
    // Display greyscale frames
    for(int i=0; i < frame_outputs.size(); i++)
        if(frame_outputs[i]->IsActive() && img[i]) {

            std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
            frame_outputs[i]->AddPoint(*latest_output, new slambench::values::FrameValue(camera_input_s[i].x, camera_input_s[i].y,
                                                                                         slambench::io::pixelformat::G_I_8,
                                                                                         (void *) (img[i]->data)));

        }

    return true;
}
