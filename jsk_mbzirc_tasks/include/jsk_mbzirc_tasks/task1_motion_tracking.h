/*
 * Copyright (c) 2016, JSK Robotics Laboratory, The University of Tokyo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef JSK_MBZIRC_TASKS_TASK1_MOTION_TRACKING_H
#define JSK_MBZIRC_TASKS_TASK1_MOTION_TRACKING_H

#include <ros/ros.h>
#include <boost/thread.hpp>

#include <tf/transform_listener.h> /* for cheating mode */
#include <geometry_msgs/PoseStamped.h> /* for gps based position and ground truth */
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <hector_quadrotor_controller/pid.h> /* for tracking pid control */

#include <boost/thread/mutex.hpp>
#if BOOST_VERSION>105200
#include <boost/thread/lock_guard.hpp>
#endif

namespace uav
{
  class MotionTracking
  {
  public:
    MotionTracking(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~MotionTracking();

    /* Stages of tracking process */
    static const uint8_t START_POINT_STAGE = 0x00;
    static const uint8_t CENTER_CROSS_STAGE = 0x01;
    static const uint8_t TRUCK_TRACKING_STAGE = 0x02;

    /* Tracking mode */
    static const uint8_t WAITING_MODE = 0x00;
    static const uint8_t AGGRESSIVE_MODE = 0x01;

    /* Control mode */
    static const uint8_t TWIST_MODE = 0x00;
    static const uint8_t ATTITUDE_MODE = 0x01;


  private:
    ros::NodeHandle nh_, nhp_;
    ros::Publisher pub_twist_cmd_; //first we use twist mode, later we should change to attitude mode
    ros::Subscriber sub_estimated_state_, sub_imu_;
    tf::TransformListener tf_; // this is for the cheat mode
    ros::Subscriber sub_ground_truth_; // this is for the cheat mode

    boost::thread tracking_thread_; //control thread
    boost::thread tf_thread_; //control thread
    boost::mutex tf_mutex_;

    int tracking_mode_;
    int control_mode_; //twist mode or attitude mode
    bool cheating_mode_; //use ground truth to go to the cross point and use ground truth to catch truck
    double level_p_gain_, level_i_gain_, level_d_gain_;
    double vetical_p_gain_, vetical_i_gain_, vetical_d_gain_;
    double center_cross_point_threshold_;
    double truck_speed_;

    uint8_t tracking_stage_;
    double velocity_limit_;
    double tracking_loop_rate_;
    double tf_loop_rate_;
    double waiting_height_, takeoff_height_, heliport_height_offset_;

    bool state_updated_flag_, imu_updated_flag_;
    bool ground_truth_updated_flag_, tf_updated_flag_;

    std::string twist_cmd_topic_name_, state_topic_name_, imu_topic_name_;
    std::string ground_truth_topic_name_;


    tf::Vector3 center_cross_point_;

    tf::Vector3 imu_acc_;
    tf::Quaternion imu_att_;
    tf::Pose estimated_state_;
    tf::Pose ground_truth_state_, heliport_state_; /* cheat mode */

    //geometry_msgs::Twist command_;
    tf::Vector3 linear_command_, anguler_command_;

    struct
    {
      hector_quadrotor_controller::PID x;
      hector_quadrotor_controller::PID y;
      hector_quadrotor_controller::PID z;
      hector_quadrotor_controller::PID yaw;
    }pid_;

    void trackingFunction();
    void tfFunction();
    void stateCallback(const nav_msgs::OdometryConstPtr &state);
    void imuCallback(const sensor_msgs::ImuConstPtr &imu);
    void groundTruthCallback(const geometry_msgs::PoseStampedConstPtr &ground_truth);

    void setGroundTruthHeliportPose(tf::StampedTransform transform);
    tf::Quaternion getGroundTruthHeliportOrientation();
    tf::Vector3 getGroundTruthHeliportOrigin();

  };

}  // mamespace uac

#endif  // JSK_MBZIRC_TASKS_TASK1_MOTION_TRACKING_H
