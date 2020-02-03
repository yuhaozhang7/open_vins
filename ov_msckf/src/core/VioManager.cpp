/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "VioManager.h"
#include "types/Landmark.h"

using namespace ov_core;
using namespace ov_msckf;

//VioManager::VioManager(ros::NodeHandle &nh) {
VioManager::VioManager(VioManagerOptions& options) {

    // Enforce that if we are doing stereo tracking, we have two cameras
    if(options.state_options.max_cameras < 1) {
        std::cerr<<"VioManager(): Specified number of cameras needs to be greater than zero\n";
        std::cerr<<"VioManager(): num cameras = " << options.state_options.max_cameras;
        std::exit(EXIT_FAILURE);
    }

    // Read in what representation our feature is

    std::transform(options.feat_rep_str.begin(), options.feat_rep_str.end(),options.feat_rep_str.begin(), ::toupper);

    // Set what representation we should be using
    if(options.feat_rep_str == "GLOBAL_3D") options.state_options.feat_representation = FeatureRepresentation::Representation::GLOBAL_3D;
    else if(options.feat_rep_str == "GLOBAL_FULL_INVERSE_DEPTH") options.state_options.feat_representation = FeatureRepresentation::Representation::GLOBAL_FULL_INVERSE_DEPTH;
    else if(options.feat_rep_str == "ANCHORED_3D") options.state_options.feat_representation = FeatureRepresentation::Representation::ANCHORED_3D;
    else if(options.feat_rep_str == "ANCHORED_FULL_INVERSE_DEPTH") options.state_options.feat_representation = FeatureRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH;
    else if(options.feat_rep_str == "ANCHORED_MSCKF_INVERSE_DEPTH") options.state_options.feat_representation = FeatureRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH;
    else {
        std::cerr<< " VioManager(): invalid feature representation specified =" << options.feat_rep_str.c_str();
        std::cerr<<"\n  VioManager(): the valid types are:";
        std::cerr<<"\n- GLOBAL_3D";
        std::cerr<<"\n- GLOBAL_FULL_INVERSE_DEPTH";
        std::cerr<<"\n- ANCHORED_3D";
        std::cerr<<"\n- ANCHORED_FULL_INVERSE_DEPTH";
        std::cerr<<"\n- ANCHORED_MSCKF_INVERSE_DEPTH";
        std::exit(EXIT_FAILURE);
    }

    // Create the state!!
    state = new State(options.state_options);

    state->calib_dt_CAMtoIMU()->set_value(options.calib_camimu_dt);
    state->calib_dt_CAMtoIMU()->set_fej(options.calib_camimu_dt);

    // Debug, print to the console!
    std::cout<<"\n=====================================";
    std::cout<<"\n  FILTER PARAMETERS:";
    std::cout<<"\n- do fej:" << options.state_options.do_fej;
    std::cout<<"\n- do imu avg: camera" << options.state_options.use_imu_avg;
    std::cout<<"\n- calibrate cam to imu: camera" << options.state_options.do_calib_camera_pose;
    std::cout<<"\n- calibrate cam intrinsics: camera" << options.state_options.do_calib_camera_intrinsics;
    std::cout<<"\n- calibrate cam imu timeoff: camera" << options.state_options.do_calib_camera_timeoffset;
    std::cout<<"\n- max clones: camera" << options.state_options.max_clone_size;
    std::cout<<"\n- max slam: camera" << options.state_options.max_slam_features;
    std::cout<<"\n- max aruco: camera" << options.state_options.max_aruco_features;
    std::cout<<"\n- max cameras: camera" << options.state_options.max_cameras;
    std::cout<<"\n- slam startup delay:" << dt_statupdelay;
    std::cout<<"\n- feature representation: " << options.feat_rep_str.c_str();


    // Debug print initial values our state use
    std::cout<<"\n=====================================";
    std::cout<<"\n  STATE INIT VALUES:";
    std::cout<<"\n- calib_camimu_dt: %.4f" << options.calib_camimu_dt;
    std::cout<<"\n- gravity:" << options.gravity;


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Debug message
    std::cout<<"\n=====================================";
    std::cout<<"\nCAMERA PARAMETERS:";

    // Loop through through, and load each of the cameras
//    TODO: make camera parameters struct
    for(int i=0; i<state->options().max_cameras; i++) {

        state->get_model_CAM(i) = options.cameras[i].is_fisheye;

        // Save this representation in our state
        state->get_intrinsics_CAM(i)->set_value(options.cameras[i].cam_calib);
        state->get_intrinsics_CAM(i)->set_fej(options.cameras[i].cam_calib);

        // Load these into our state
        Eigen::Matrix<double,7,1> cam_eigen;
        cam_eigen.block(0,0,4,1) = rot_2_quat(options.cameras[i].T_CtoI.block(0,0,3,3).transpose());
        cam_eigen.block(4,0,3,1) = -options.cameras[i].T_CtoI.block(0,0,3,3).transpose()*options.cameras[i].T_CtoI.block(0,3,3,1);
        state->get_calib_IMUtoCAM(i)->set_value(cam_eigen);
        state->get_calib_IMUtoCAM(i)->set_fej(cam_eigen);

        // Append to our maps for our feature trackers
        camera_fisheye.insert({i,options.cameras[i].is_fisheye});
        camera_calib.insert({i,options.cameras[i].cam_calib});
        camera_wh.insert({i,options.cameras[i].wh});

        // Debug printing
        cout << "cam_" << i << "wh:" << endl << options.cameras[i].wh.first << " x " << options.cameras[i].wh.second << endl;
        cout << "cam_" << i << "K:" << endl << options.cameras[i].cam_calib.block(0,0,4,1).transpose() << endl;
        cout << "cam_" << i << "d:" << endl << options.cameras[i].cam_calib.block(4,0,4,1).transpose() << endl;
        cout << "T_C" << i << "toI:" << endl << options.cameras[i].T_CtoI << endl << endl;

    }

    // Debug message
    std::cout<<"=====================================";

    //===================================================================================
    //===================================================================================
    //===================================================================================


    // Debug, print to the console!
    std::cout<<"\n  FEATURE INITIALIZER PARAMETERS:";
    std::cout<<"\n- max runs: " << options.featinit_options.max_runs;
    std::cout<<"\n- init lambda: " << options.featinit_options.init_lamda;
    std::cout<<"\n- max lambda: " << options.featinit_options.max_lamda;
    std::cout<<"\n- min final dx: " << options.featinit_options.min_dx;
    std::cout<<"\n- min delta cost: " << options.featinit_options.min_dcost;
    std::cout<<"\n- lambda multiple: " << options.featinit_options.lam_mult;
    std::cout<<"\n- closest feature dist: " << options.featinit_options.min_dist;
    std::cout<<"\n- furthest feature dist: " << options.featinit_options.max_dist;
    std::cout<<"\n- max baseline ratio: " << options.featinit_options.max_baseline;
    std::cout<<"\n- max condition number: " << options.featinit_options.max_cond_number;

    std::cout<<"\n  TRACKING PARAMETERS:";
    std::cout<<"\n- use klt: " << options.use_klt;
    std::cout<<"\n- use aruco: " << options.use_aruco;
    std::cout<<"\n- max track features: " << options.num_pts;
    std::cout<<"\n- max aruco tags: " << state->options().max_aruco_features;
    std::cout<<"\n- grid size: " << options.grid_x << " x " << options.grid_y;
    std::cout<<"\n- fast threshold: " << options.fast_threshold;
    std::cout<<"\n- min pixel distance: " << options.min_px_dist;
    std::cout<<"\n- downsize aruco image: " << options.downsize_aruco;


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Debug print out
    std::cout<<"\n  PROPAGATOR NOISES:";
    std::cout<<"\n- sigma_w: " << options.imu_noises.sigma_w;
    std::cout<<"\n- sigma_a: " << options.imu_noises.sigma_a;
    std::cout<<"\n- sigma_wb: " << options.imu_noises.sigma_wb;
    std::cout<<"\n- sigma_ab: " << options.imu_noises.sigma_ab;



    // Debug print out
    std::cout<<"\n  INITIALIZATION PARAMETERS:";
    std::cout<<"\n- init_window_time: " << options.init_window_time;
    std::cout<<"\n- init_imu_thresh: " << options.init_imu_thresh;



    // If downsampling aruco, then double our noise values
    options.aruco_options.sigma_pix = (options.downsize_aruco) ? 2*options.aruco_options.sigma_pix : options.aruco_options.sigma_pix;

    std::cout<<"\nMSCKFUPDATER PARAMETERS:";
    std::cout<<"\n- sigma_pxmsckf: " << options.msckf_options.sigma_pix;
    std::cout<<"\n- sigma_pxslam: " << options.slam_options.sigma_pix;
    std::cout<<"\n- sigma_pxaruco: " << options.aruco_options.sigma_pix;
    std::cout<<"\n- chi2_multipler msckf: " << options.msckf_options.chi2_multipler;
    std::cout<<"\n- chi2_multipler slam: " << options.slam_options.chi2_multipler;
    std::cout<<"\n- chi2_multipler aruco: " << options.aruco_options.chi2_multipler;


    //===================================================================================
    //===================================================================================
    //===================================================================================


    // Lets make a feature extractor
    if(options.use_klt) {
        trackFEATS = new TrackKLT(options.num_pts,state->options().max_aruco_features,options.fast_threshold,options.grid_x,options.grid_y,options.min_px_dist);
        trackFEATS->set_calibration(camera_calib, camera_fisheye);
    } else {
        trackFEATS = new TrackDescriptor(options.num_pts,state->options().max_aruco_features,options.fast_threshold,options.grid_x,options.grid_y,options.knn_ratio);
        trackFEATS->set_calibration(camera_calib, camera_fisheye);
    }

    // Initialize our aruco tag extractor
    if(options.use_aruco) {
        trackARUCO = new TrackAruco(state->options().max_aruco_features,options.downsize_aruco);
        trackARUCO->set_calibration(camera_calib, camera_fisheye);
    }

    // Initialize our state propagator
    propagator = new Propagator(options.imu_noises,options.gravity);

    // Our state initialize
    initializer = new InertialInitializer(options.gravity,options.init_window_time, options.init_imu_thresh);

    // Make the updater!
    updaterMSCKF = new UpdaterMSCKF(options.msckf_options, options.featinit_options);
    updaterSLAM = new UpdaterSLAM(options.slam_options, options.aruco_options, options.featinit_options);
}

void VioManager::feed_measurement_imu(double timestamp, Eigen::Vector3d wm, Eigen::Vector3d am) {

    // Push back to our propagator
    propagator->feed_imu(timestamp,wm,am);

    // Push back to our initializer
    if(!is_initialized_vio) {
        initializer->feed_imu(timestamp, wm, am);
    }

}

void VioManager::feed_measurement_monocular(double timestamp, cv::Mat& img0, size_t cam_id) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Feed our trackers
    trackFEATS->feed_monocular(timestamp, img0, cam_id);

    // If aruoc is avalible, the also pass to it
    if(trackARUCO != nullptr) {
        trackARUCO->feed_monocular(timestamp, img0, cam_id);
    }
    rT2 =  boost::posix_time::microsec_clock::local_time();

    // If we do not have VIO initialization, then try to initialize
    // TODO: Or if we are trying to reset the system, then do that here!
    if(!is_initialized_vio) {
        is_initialized_vio = try_to_initialize();
        if(!is_initialized_vio) return;
    }

    // Call on our propagate and update function
    do_feature_propagate_update(timestamp);


}

void VioManager::feed_measurement_stereo(double timestamp, cv::Mat& img0, cv::Mat& img1, size_t cam_id0, size_t cam_id1) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Assert we have good ids
    assert(cam_id0!=cam_id1);

    // Feed our stereo trackers, if we are not doing binocular
    if(use_stereo) {
        trackFEATS->feed_stereo(timestamp, img0, img1, cam_id0, cam_id1);
    } else {
        boost::thread t_l = boost::thread(&TrackBase::feed_monocular, trackFEATS, boost::ref(timestamp), boost::ref(img0), boost::ref(cam_id0));
        boost::thread t_r = boost::thread(&TrackBase::feed_monocular, trackFEATS, boost::ref(timestamp), boost::ref(img1), boost::ref(cam_id1));
        t_l.join();
        t_r.join();
    }

    // If aruoc is avalible, the also pass to it
    // NOTE: binocular tracking for aruco doesn't make sense as we by default have the ids
    // NOTE: thus we just call the stereo tracking if we are doing binocular!
    if(trackARUCO != nullptr) {
        trackARUCO->feed_stereo(timestamp, img0, img1, cam_id0, cam_id1);
    }
    rT2 =  boost::posix_time::microsec_clock::local_time();

    // If we do not have VIO initialization, then try to initialize
    // TODO: Or if we are trying to reset the system, then do that here!
    if(!is_initialized_vio) {
        is_initialized_vio = try_to_initialize();
        if(!is_initialized_vio) return;
    }

    // Call on our propagate and update function
    do_feature_propagate_update(timestamp);

}

void VioManager::feed_measurement_simulation(double timestamp, const std::vector<int> &camids, const std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> &feats) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Check if we actually have a simulated tracker
    TrackSIM *trackSIM = dynamic_cast<TrackSIM*>(trackFEATS);
    if(trackSIM == nullptr) {
        //delete trackFEATS; //(fix this error in the future)
        trackFEATS = new TrackSIM(state->options().max_aruco_features);
        trackFEATS->set_calibration(camera_calib, camera_fisheye);
        std::cerr<<("[SIM]: casting our tracker to a TrackSIM object!");
    }

    // Cast the tracker to our simulation tracker
    trackSIM = dynamic_cast<TrackSIM*>(trackFEATS);
    trackSIM->set_width_height(camera_wh);

    // Feed our simulation tracker
    trackSIM->feed_measurement_simulation(timestamp, camids, feats);
    rT2 =  boost::posix_time::microsec_clock::local_time();

    // If we do not have VIO initialization, then return an error
    if(!is_initialized_vio) {
        std::cerr<<("[SIM]: your vio system should already be initialized before simulating features!!!");
        std::cerr<<("[SIM]: initialize your system first before calling feed_measurement_simulation()!!!!");
        std::exit(EXIT_FAILURE);
    }

    // Call on our propagate and update function
    do_feature_propagate_update(timestamp);
}

bool VioManager::try_to_initialize() {

    // Returns from our initializer
    double time0;
    Eigen::Matrix<double, 4, 1> q_GtoI0;
    Eigen::Matrix<double, 3, 1> b_w0, v_I0inG, b_a0, p_I0inG;

    // Try to initialize the system
    bool success = initializer->initialize_with_imu(time0, q_GtoI0, b_w0, v_I0inG, b_a0, p_I0inG);

    // Return if it failed
    if (!success) {
        return false;
    }

    // Make big vector (q,p,v,bg,ba), and update our state
    // Note: start from zero position, as this is what our covariance is based off of
    Eigen::Matrix<double,16,1> imu_val;
    imu_val.block(0,0,4,1) = q_GtoI0;
    imu_val.block(4,0,3,1) << 0,0,0;
    imu_val.block(7,0,3,1) = v_I0inG;
    imu_val.block(10,0,3,1) = b_w0;
    imu_val.block(13,0,3,1) = b_a0;
    //imu_val.block(10,0,3,1) << 0,0,0;
    //imu_val.block(13,0,3,1) << 0,0,0;
    state->imu()->set_value(imu_val);
    state->set_timestamp(time0);

    // Else we are good to go, print out our stats
    std::cout<<("\033[0;32m[INIT]: orientation = << , << , << , << \033[0m",state->imu()->quat()(0),state->imu()->quat()(1),state->imu()->quat()(2),state->imu()->quat()(3));
    std::cout<<("\033[0;32m[INIT]: bias gyro = << , << , << \033[0m",state->imu()->bias_g()(0),state->imu()->bias_g()(1),state->imu()->bias_g()(2));
    std::cout<<("\033[0;32m[INIT]: velocity = << , << , << \033[0m",state->imu()->vel()(0),state->imu()->vel()(1),state->imu()->vel()(2));
    std::cout<<("\033[0;32m[INIT]: bias accel = << , << , << \033[0m",state->imu()->bias_a()(0),state->imu()->bias_a()(1),state->imu()->bias_a()(2));
    std::cout<<("\033[0;32m[INIT]: position = << , << , << \033[0m",state->imu()->pos()(0),state->imu()->pos()(1),state->imu()->pos()(2));
    return true;

}

void VioManager::do_feature_propagate_update(double timestamp) {


    //===================================================================================
    // State propagation, and clone augmentation
    //===================================================================================

    // Return if the camera measurement is out of order
    if(state->timestamp() >= timestamp) {
//        ROS_WARN("image received out of order (prop dt = %3f)",(timestamp-state->timestamp()));
        return;
    }

    // If we have just started up, we should record this time as the current time
    if(startup_time == -1) {
        startup_time = timestamp;
    }

    // Propagate the state forward to the current update time
    // Also augment it with a new clone!
    propagator->propagate_and_clone(state, timestamp);
    rT3 =  boost::posix_time::microsec_clock::local_time();

    // If we have not reached max clones, we should just return...
    // This isn't super ideal, but it keeps the logic after this easier...
    // We can start processing things when we have at least 5 clones since we can start triangulating things...
    if((int)state->n_clones() < std::min(state->options().max_clone_size,5)) {
        std::cout<<"waiting for enough clone states (camera " <<  (int)state->n_clones() << " of " << std::min(state->options().max_clone_size,5) << ") ....";
        return;
    }

    // Return if we where unable to propagate
    if(state->timestamp() != timestamp) {
        std::cerr<<"[PROP]: Propagator unable to propagate the state forward in time!" << std::endl;
        std::cerr<<"[PROP]: It has been %.3f since last time we propagated " << timestamp-state->timestamp();
        return;
    }

    //===================================================================================
    // MSCKF features and KLT tracks that are SLAM features
    //===================================================================================


    // Now, lets get all features that should be used for an update that are lost in the newest frame
    std::vector<Feature*> feats_lost, feats_marg, feats_slam;
    feats_lost = trackFEATS->get_feature_database()->features_not_containing_newer(state->timestamp());

    // Don't need to get the oldest features untill we reach our max number of clones
    if((int)state->n_clones() > state->options().max_clone_size) {
        feats_marg = trackFEATS->get_feature_database()->features_containing(state->margtimestep());
        if(trackARUCO != nullptr && timestamp-startup_time >= dt_statupdelay) {
            feats_slam = trackARUCO->get_feature_database()->features_containing(state->margtimestep());
        }
    }

    // We also need to make sure that the max tracks does not contain any lost features
    // This could happen if the feature was lost in the last frame, but has a measurement at the marg timestep
    auto it1 = feats_lost.begin();
    while(it1 != feats_lost.end()) {
        if(std::find(feats_marg.begin(),feats_marg.end(),(*it1)) != feats_marg.end()) {
            //ROS_WARN("FOUND FEATURE THAT WAS IN BOTH feats_lost and feats_marg!!!!!!");
            it1 = feats_lost.erase(it1);
        } else {
            it1++;
        }
    }

    // Find tracks that have reached max length, these can be made into SLAM features
    std::vector<Feature*> feats_maxtracks;
    auto it2 = feats_marg.begin();
    while(it2 != feats_marg.end()) {
        // See if any of our camera's reached max track
        bool reached_max = false;
        for (const auto &cams: (*it2)->timestamps){
            if ((int)cams.second.size() > state->options().max_clone_size){
                reached_max = true;
                break;
            }
        }
        // If max track, then add it to our possible slam feature list
        if(reached_max) {
            feats_maxtracks.push_back(*it2);
            it2 = feats_marg.erase(it2);
        } else {
            it2++;
        }
    }

    // Count how many aruco tags we have in our state
    int curr_aruco_tags = 0;
    auto it0 = state->features_SLAM().begin();
    while(it0 != state->features_SLAM().end()) {
        if ((int) (*it0).second->_featid <= state->options().max_aruco_features) curr_aruco_tags++;
        it0++;
    }

    // Append a new SLAM feature if we have the room to do so
    // Also check that we have waited our delay amount (normally prevents bad first set of slam points)
    if(state->options().max_slam_features > 0 && timestamp-startup_time >= dt_statupdelay && (int)state->features_SLAM().size() < state->options().max_slam_features+curr_aruco_tags) {
        // Get the total amount to add, then the max amount that we can add given our marginalize feature array
        int amount_to_add = (state->options().max_slam_features+curr_aruco_tags)-(int)state->features_SLAM().size();
        int valid_amount = (amount_to_add > (int)feats_maxtracks.size())? (int)feats_maxtracks.size() : amount_to_add;
        // If we have at least 1 that we can add, lets add it!
        // Note: we remove them from the feat_marg array since we don't want to reuse information...
        if(valid_amount > 0) {
            feats_slam.insert(feats_slam.end(), feats_maxtracks.end()-valid_amount, feats_maxtracks.end());
            feats_maxtracks.erase(feats_maxtracks.end()-valid_amount, feats_maxtracks.end());
        }
    }

    // Loop through current SLAM features, we have tracks of them, grab them for this update!
    // Note: if we have a slam feature that has lost tracking, then we should marginalize it out
    // Note: if you do not use FEJ, these types of slam features *degrade* the estimator performance....
    for (std::pair<const size_t, Landmark*> &landmark : state->features_SLAM()) {
        if(trackARUCO != nullptr) {
            Feature* feat1 = trackARUCO->get_feature_database()->get_feature(landmark.second->_featid);
            if(feat1 != nullptr) feats_slam.push_back(feat1);
        }
        Feature* feat2 = trackFEATS->get_feature_database()->get_feature(landmark.second->_featid);
        if(feat2 != nullptr) feats_slam.push_back(feat2);
        if(feat2 == nullptr) landmark.second->should_marg = true;
    }

    // Lets marginalize out all old SLAM features here
    // These are ones that where not successfully tracked into the current frame
    // We do *NOT* marginalize out our aruco tags
    StateHelper::marginalize_slam(state);

    // Separate our SLAM features into new ones, and old ones
    std::vector<Feature*> feats_slam_DELAYED, feats_slam_UPDATE;
    for(size_t i=0; i<feats_slam.size(); i++) {
        if(state->features_SLAM().find(feats_slam.at(i)->featid) != state->features_SLAM().end()) {
            feats_slam_UPDATE.push_back(feats_slam.at(i));
//            std::cout<<("[UPDATE-SLAM]: found old feature camera (camera measurements)",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
        } else {
            feats_slam_DELAYED.push_back(feats_slam.at(i));
//            std::cout<<("[UPDATE-SLAM]: new feature ready camera (camera measurements)",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
        }
    }

    // Concatenate our MSCKF feature arrays (i.e., ones not being used for slam updates)
    std::vector<Feature*> featsup_MSCKF = feats_lost;
    featsup_MSCKF.insert(featsup_MSCKF.end(), feats_marg.begin(), feats_marg.end());
    featsup_MSCKF.insert(featsup_MSCKF.end(), feats_maxtracks.begin(), feats_maxtracks.end());


    //===================================================================================
    // Now that we have a list of features, lets do the EKF update for MSCKF and SLAM!
    //===================================================================================

    // Pass them to our MSCKF updater
    // We update first so that our SLAM initialization will be more accurate??
    updaterMSCKF->update(state, featsup_MSCKF);
    rT4 =  boost::posix_time::microsec_clock::local_time();

    // Perform SLAM delay init and update
    updaterSLAM->update(state, feats_slam_UPDATE);
    updaterSLAM->delayed_init(state, feats_slam_DELAYED);
    rT5 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    // Update our visualization feature set, and clean up the old features
    //===================================================================================


    // Save all the MSCKF features used in the update
    good_features_MSCKF.clear();
    for(Feature* feat : featsup_MSCKF) {
        good_features_MSCKF.push_back(feat->p_FinG);
        feat->to_delete = true;
    }

    // Remove features that where used for the update from our extractors at the last timestep
    // This allows for measurements to be used in the future if they failed to be used this time
    // Note we need to do this before we feed a new image, as we want all new measurements to NOT be deleted
    trackFEATS->get_feature_database()->cleanup();
    if(trackARUCO != nullptr) {
        trackARUCO->get_feature_database()->cleanup();
    }

    //===================================================================================
    // Cleanup, marginalize out what we don't need any more...
    //===================================================================================

    // First do anchor change if we are about to lose an anchor pose
    updaterSLAM->change_anchors(state);

    // Marginalize the oldest clone of the state if we are at max length
    if((int)state->n_clones() > state->options().max_clone_size) {
        StateHelper::marginalize_old_clone(state);
    }

    // Finally if we are optimizing our intrinsics, update our trackers
    if(state->options().do_calib_camera_intrinsics) {
        // Get vectors arrays
        std::map<size_t, Eigen::VectorXd> cameranew_calib;
        std::map<size_t, bool> cameranew_fisheye;
        for(int i=0; i<state->options().max_cameras; i++) {
            Vec* calib = state->get_intrinsics_CAM(i);
            bool isfish = state->get_model_CAM(i);
            cameranew_calib.insert({i,calib->value()});
            cameranew_fisheye.insert({i,isfish});
        }
        // Update the trackers and their databases
        trackFEATS->set_calibration(cameranew_calib, cameranew_fisheye, true);
        if(trackARUCO != nullptr) {
            trackARUCO->set_calibration(cameranew_calib, cameranew_fisheye, true);
        }
    }
    rT6 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    // Debug info, and stats tracking
    //===================================================================================


    // Timing information
    std::cout<<"\u001b[34m[TIME]:" <<  (rT2-rT1).total_microseconds() * 1e-6 << "seconds for tracking\u001b[0m";
    std::cout<<"\u001b[34m[TIME]:" <<  "seconds for propagation\u001b[0m" << (rT3-rT2).total_microseconds() * 1e-6;
    std::cout<<"\u001b[34m[TIME]:" <<  "seconds for MSCKF update (camera features)\u001b[0m" << (rT4-rT3).total_microseconds() * 1e-6
                                   <<  "Size:" <<  (int)good_features_MSCKF.size();
    if(state->options().max_slam_features > 0)
        std::cout<<"\u001b[34m[TIME]:" <<  (rT5-rT4).total_microseconds() * 1e-6 << " "
                                       << (int)feats_slam_DELAYED.size() << " "
                                       << (int)feats_slam_UPDATE.size() << " "
                                       << "seconds for SLAM update (camera delayed, camera update)\u001b[0m" << std::endl;
    std::cout<<"\u001b[34m[TIME]:" <<  "seconds for marginalization\u001b[0m"
                                   << (rT6-rT5).total_microseconds() * 1e-6
                                   << "camera clones in state" << (int)state->n_clones();
    std::cout<<"\u001b[34m[TIME]:" <<  "seconds for total\u001b[0m" << (rT6-rT1).total_microseconds() * 1e-6;

    // Update our distance traveled
    if(timelastupdate != -1 && state->get_clones().find(timelastupdate) != state->get_clones().end()) {
        Eigen::Matrix<double,3,1> dx = state->imu()->pos() - state->get_clone(timelastupdate)->pos();
        distance += dx.norm();
    }
    timelastupdate = timestamp;

    // Debug, print our current state
    printf("q_GtoI = %.3f,%.3f,%.3f,%.3f | p_IinG = %.3f,%.3f,%.3f | dist = %.2f (meters)",
            state->imu()->quat()(0),state->imu()->quat()(1),state->imu()->quat()(2),state->imu()->quat()(3),
            state->imu()->pos()(0),state->imu()->pos()(1),state->imu()->pos()(2),distance);
    std::cout<<"bg = " << state->imu()->bias_g()(0)
                       << state->imu()->bias_g()(1)
                       << state->imu()->bias_g()(2)
                       << "| ba ="
                       << state->imu()->bias_a()(0)
                       << state->imu()->bias_a()(1)
                       << state->imu()->bias_a()(2) << std::endl;


    // Debug for camera imu offset
//    if(state->options().do_calib_camera_timeoffset) {
    std::cout<<"camera-imu timeoffset = " << state->calib_dt_CAMtoIMU()->value()(0);
//    }

    // Debug for camera intrinsics
    if(state->options().do_calib_camera_intrinsics) {
        for(int i=0; i<state->options().max_cameras; i++) {
            Vec* calib = state->get_intrinsics_CAM(i);
            printf("camera intrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f,%.3f",(int)i,
                    calib->value()(0),calib->value()(1),calib->value()(2),calib->value()(3),
                    calib->value()(4),calib->value()(5),calib->value()(6),calib->value()(7));
        }
    }

    // Debug for camera extrinsics
    if(state->options().do_calib_camera_pose) {
        for(int i=0; i<state->options().max_cameras; i++) {
            PoseJPL* calib = state->get_calib_IMUtoCAM(i);
            printf("camera extrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f",(int)i,
                    calib->quat()(0),calib->quat()(1),calib->quat()(2),calib->quat()(3),
                    calib->pos()(0),calib->pos()(1),calib->pos()(2));
        }
    }
}


