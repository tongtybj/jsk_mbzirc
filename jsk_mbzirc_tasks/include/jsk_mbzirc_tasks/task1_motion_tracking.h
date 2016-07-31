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
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

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
    static const uint8_t IDLE_STAGE = 0x00;
    static const uint8_t START_POINT_STAGE = 0x01;
    static const uint8_t CENTER_CROSS_STAGE = 0x02;
    static const uint8_t TRUCK_TRACKING_STAGE = 0x03;

    /* Tracking mode */
    static const uint8_t WAITING_MODE = 0x00;
    static const uint8_t AGGRESSIVE_MODE = 0x01;

    /* Control mode */
    static const uint8_t TWIST_MODE = 0x00;
    static const uint8_t ATTITUDE_MODE = 0x01;


  private:
    ros::NodeHandle nh_, nhp_;
    ros::Publisher pub_twist_cmd_; //first we use twist mode, later we should change to attitude mode
    ros::Subscriber sub_uav_state_, sub_uav_imu_, sub_heliport_pos_;
    tf::TransformListener tf_; // this is for the cheat mode
    ros::Subscriber sub_uav_ground_truth_; // this is for the cheat mode
    ros::Subscriber sub_truck_ground_truth_; // this is for the cheat mode
    ros::ServiceClient motor_engage_client_, motor_shutdown_client_;
    ros::ServiceServer tracking_start_service_srv_;

    boost::thread tracking_thread_; //control thread
    boost::mutex heliport_mutex_;
    boost::mutex uav_mutex_;

    int tracking_mode_;
    int control_mode_; //twist mode or attitude mode
    bool cheating_mode_uav_, cheating_mode_heliport_; //use ground truth to go to the cross point and use ground truth to catch truck
    double level_p_gain_, level_i_gain_, level_d_gain_;
    double vetical_p_gain_, vetical_i_gain_, vetical_d_gain_;
    double center_cross_point_threshold_;
    double truck_speed_;

    uint8_t tracking_stage_;
    double velocity_limit_;
    double descending_velocity_limit_;
    double tracking_loop_rate_;
    double tf_loop_rate_;
    double waiting_height_, takeoff_height_, heliport_height_;

    bool uav_state_updated_flag_, uav_imu_updated_flag_;
    bool uav_ground_truth_updated_flag_, truck_updated_flag_, heliport_pos_updated_flag_;

    double landing_level_distance_threshold_, landing_vertical_distance_threshold_, landing_vertical_distance_threshold2_, landing_vertical_velocity_;

    std::string twist_cmd_topic_name_, uav_state_topic_name_, uav_imu_topic_name_, heliport_pos_topic_name_;
    std::string uav_ground_truth_topic_name_, truck_ground_truth_topic_name_;

    tf::Vector3 center_cross_point_;

    tf::Vector3 uav_imu_acc_;
    tf::Quaternion uav_imu_att_;
    tf::Pose uav_state_, heliport_state_;

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

    void uavStateCallback(const nav_msgs::OdometryConstPtr &state);
    void uavImuCallback(const sensor_msgs::ImuConstPtr &imu);
    void heliportCallback(const geometry_msgs::PointStampedConstPtr &state);

    /* ground truth for cheat mode */
    void uavGroundTruthCallback(const geometry_msgs::PoseStampedConstPtr &ground_truth);
    void truckGroundTruthCallback(const nav_msgs::OdometryConstPtr &ground_truth);

    void setUavPose(tf::Vector3 origin, tf::Quaternion q)
    {
      boost::mutex::scoped_lock lock(uav_mutex_);
      uav_state_.setOrigin(origin);
      uav_state_.setRotation(q);
    }

    tf::Pose getUavPose()
    {
      boost::mutex::scoped_lock lock(uav_mutex_);
      return uav_state_;
    }

    tf::Quaternion getUavOrientation()
    {
      boost::mutex::scoped_lock lock(uav_mutex_);
      return uav_state_.getRotation();
    }

    tf::Vector3 getUavOrigin()
    {
      boost::mutex::scoped_lock lock(uav_mutex_);
      return uav_state_.getOrigin();
    }

    void setHeliportPose(tf::Vector3 origin, tf::Quaternion q)
    {
      boost::mutex::scoped_lock lock(heliport_mutex_);

      heliport_state_.setOrigin(origin);
      heliport_state_.setRotation(q);
    }

    void setHeliportPosition(tf::Vector3 origin)
    {
      boost::mutex::scoped_lock lock(heliport_mutex_);
      heliport_state_.setOrigin(origin);
    }
     
    void setHeliportOrientation(tf::Quaternion q)
    {
      boost::mutex::scoped_lock lock(heliport_mutex_);
      heliport_state_.setRotation(q);
    }

    tf::Quaternion getHeliportOrientation()
    {
      boost::mutex::scoped_lock lock(heliport_mutex_);
      return heliport_state_.getRotation();
    }

    tf::Vector3 getHeliportOrigin()
    {
      boost::mutex::scoped_lock lock(heliport_mutex_);
      return heliport_state_.getOrigin();
    }

    bool startTrackingCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
      std_srvs::Empty srv;
      while(1)
        {
          if(motor_engage_client_.call(srv)) break;
        }

      tracking_stage_ = START_POINT_STAGE;
      uav_state_updated_flag_ = false;
      uav_imu_updated_flag_ = false;
      heliport_pos_updated_flag_ = false;
      center_cross_point_.setZ(waiting_height_);
    }

  };

}  // mamespace uac

#endif  // JSK_MBZIRC_TASKS_TASK1_MOTION_TRACKING_H
