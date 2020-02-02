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

#include "core/VioManager.h"
#include "core/RosVisualizer.h"
#include "utils/dataset_reader.h"


using namespace ov_msckf;


VioManager* sys;
RosVisualizer* viz;


// Main function
int main(int argc, char** argv)
{

    // Launch our ros node
//    ros::init(argc, argv, "run_serial_msckf");
//    ros::NodeHandle nh("~");

    // Create our VIO system
    sys = new VioManager();
//    viz = new RosVisualizer(nh, sys);


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Our camera topics (left and right stereo)
    std::string topic_imu;
    std::string topic_camera0;
    std::string topic_camera1;

    // Load groundtruth if we have it
    std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
//    if (nh.hasParam("path_gt")) {
//        std::string path_to_gt;
//        nh.param<std::string>("path_gt", path_to_gt, "");
//        DatasetReader::load_gt_file(path_to_gt, gt_states);
//        ROS_INFO("gt file path is: %s", path_to_gt.c_str());
//    }

    // Read in what mode we should be processing in (1=mono, 2=stereo)
    int max_cameras;


    //===================================================================================
    //===================================================================================
    //===================================================================================


    // Start a few seconds in from the full view time
    // If we have a negative duration then use the full bag length
//    view_full.addQuery(bag);
//    ros::Time time_init = view_full.getBeginTime();
//    time_init += ros::Duration(bag_start);
//    ros::Time time_finish = (bag_durr < 0)? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
//    ROS_INFO("time start = %.6f", time_init.toSec());
//    ROS_INFO("time end   = %.6f", time_finish.toSec());


    // Buffer variables for our system (so we always have imu to use)
    bool has_left = false;
    bool has_right = false;
    cv::Mat img0, img1;
    cv::Mat img0_buffer, img1_buffer;
//    double time = time_init.toSec();
//    double time_buffer = time_init.toSec();
    std::list<cv::Mat> images;

    //===================================================================================
    //===================================================================================
    //===================================================================================


    // Step through the rosbag
    for (const auto& m : images) {

        // Handle IMU measurement
//        sensor_msgs::Imu::ConstPtr s2 = m.instantiate<sensor_msgs::Imu>();
        if (s2 != NULL && m.getTopic() == topic_imu) {
            // convert into correct format
//            double timem = (*s2).header.stamp.toSec();
//            get timestamp

            Eigen::Matrix<double, 3, 1> wm, am;
            wm << (*s2).angular_velocity.x, (*s2).angular_velocity.y, (*s2).angular_velocity.z;
            am << (*s2).linear_acceleration.x, (*s2).linear_acceleration.y, (*s2).linear_acceleration.z;
            // send time, angular velocity, linear acceleration
            sys->feed_measurement_imu(timem, wm, am);
        }

        // Handle LEFT camera
        sensor_msgs::Image::ConstPtr s0 = m.instantiate<sensor_msgs::Image>();
        if (s0 != NULL && m.getTopic() == topic_camera0) {
            // Get the image
            // Save to our temp variable
            has_left = true;
            img0 = cv_ptr->image.clone();
            time = cv_ptr->header.stamp.toSec();
        }

        // Handle RIGHT camera
        sensor_msgs::Image::ConstPtr s1 = m.instantiate<sensor_msgs::Image>();
        if (s1 != NULL && m.getTopic() == topic_camera1) {
            // Get the image
            // Save to our temp variable (use a right image that is near in time)
            // TODO: fix this logic as the left will still advance instead of waiting
            // TODO: should implement something like here:
            // TODO: https://github.com/rpng/MARS-VINS/blob/master/example_ros/ros_driver.cpp
            //if(std::abs(cv_ptr->header.stamp.toSec()-time) < 0.02) {
            has_right = true;
            img1 = cv_ptr->image.clone();
        }


        // Fill our buffer if we have not
        if(has_left && img0_buffer.rows == 0) {
            has_left = false;
            time_buffer = time;
            img0_buffer = img0.clone();
        }

        // Fill our buffer if we have not
        if(has_right && img1_buffer.rows == 0) {
            has_right = false;
            img1_buffer = img1.clone();
        }


        // If we are in monocular mode, then we should process the left if we have it
        if(max_cameras==1 && has_left) {
            // process once we have initialized with the GT
            Eigen::Matrix<double, 17, 1> imustate;
            if(!gt_states.empty() && !sys->intialized() && DatasetReader::get_gt_state(time_buffer,imustate,gt_states)) {
                //biases are pretty bad normally, so zero them
                //imustate.block(11,0,6,1).setZero();
                sys->initialize_with_gt(imustate);
            } else if(gt_states.empty() || sys->intialized()) {
                sys->feed_measurement_monocular(time_buffer, img0_buffer, 0);
            }
            // visualize
            viz->visualize();
            // reset bools
            has_left = false;
            // move buffer forward
            time_buffer = time;
            img0_buffer = img0.clone();
        }


        // If we are in stereo mode and have both left and right, then process
//        if(max_cameras==2 && has_left && has_right) {
//            // process once we have initialized with the GT
//            Eigen::Matrix<double, 17, 1> imustate;
//            if(!gt_states.empty() && !sys->intialized() && DatasetReader::get_gt_state(time_buffer,imustate,gt_states)) {
//                //biases are pretty bad normally, so zero them
//                //imustate.block(11,0,6,1).setZero();
//                sys->initialize_with_gt(imustate);
//            } else if(gt_states.empty() || sys->intialized()) {
//                sys->feed_measurement_stereo(time_buffer, img0_buffer, img1_buffer, 0, 1);
//            }
//            // visualize
//            viz->visualize();
//            // reset bools
//            has_left = false;
//            has_right = false;
//            // move buffer forward
//            time_buffer = time;
//            img0_buffer = img0.clone();
//            img1_buffer = img1.clone();
//        }

    }

    // Final visualization
//    viz->visualize_final();

    // Finally delete our system
    delete sys;
//    delete viz;


    // Done!
    return EXIT_SUCCESS;

}
