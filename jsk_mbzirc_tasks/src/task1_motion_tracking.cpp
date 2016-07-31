#include <jsk_mbzirc_tasks/task1_motion_tracking.h>

namespace uav
{

  MotionTracking::MotionTracking(ros::NodeHandle nh, ros::NodeHandle nhp):
    nh_(nh), nhp_(nhp)
  {
    nhp_.param("tracking_mode", tracking_mode_, 0);
    nhp_.param("control_mode", control_mode_, 0);
    nhp_.param("cheating_mode_uav", cheating_mode_uav_, true);
    nhp_.param("cheating_mode_heliport", cheating_mode_heliport_, true);

    nhp_.param("waiting_height", waiting_height_, 8.0); // 8m
    nhp_.param("takeoff_height", takeoff_height_, 0.0); // 8m
    nhp_.param("heliport_height", heliport_height_, 1.1); // 8m

    nhp_.param("truck_speed", truck_speed_, 4.0); // 15Km/h = 4.16666666m/s
    nhp_.param("velocity_limit", velocity_limit_, 8.0); // 30Km/h = 8.33333m/s
    nhp_.param("descending_velocity_limit", descending_velocity_limit_, -1.0); // 30Km/h = 8.33333m/s
    nhp_.param("tracking_loop_rate", tracking_loop_rate_, 50.0);
    nhp_.param("center_cross_point_threshold", center_cross_point_threshold_, 1.0);

    nhp_.param("twist_cmd_topic_name", twist_cmd_topic_name_, std::string("command/twist"));
    nhp_.param("uav_state_topic_name", uav_state_topic_name_, std::string("state"));
    nhp_.param("uav_imu_topic_name", uav_imu_topic_name_, std::string("imu"));
    nhp_.param("heliport_pos_topic_name", heliport_pos_topic_name_, std::string("/uav_landing_region/output/point"));

    nhp_.param("landing_level_distance_threshold", landing_level_distance_threshold_, 0.6);
    nhp_.param("landing_vertical_distance_threshold", landing_vertical_distance_threshold_, 0.2);
    nhp_.param("landing_vertical_distance_threshold2", landing_vertical_distance_threshold2_, 0.05);
    nhp_.param("landing_vertical_velocity", landing_vertical_velocity_, -0.5);

    /* cheating mode */
    nhp_.param("uav_ground_truth_topic_name", uav_ground_truth_topic_name_, std::string("ground_truth_to_tf/pose"));
    nhp_.param("truck_ground_truth_topic_name", truck_ground_truth_topic_name_, std::string("/truck/ground_truth/odom"));

    /*
      nhp_.param("level_p_gain", level_p_gain_, 1.0);
      nhp_.param("level_i_gain", level_i_gain_, 0.0001);
      nhp_.param("level_d_gain", level_d_gain_, 0.1);

      nhp_.param("vertical_p_gain", vertical_p_gain_, 1.0);
      nhp_.param("vertical_i_gain", vertical_i_gain_, 0.0001);
      nhp_.param("vertical_d_gain", vertical_d_gain_, 0.1);
    */

    pid_.x.init(ros::NodeHandle(nhp, "x"));
    pid_.y.init(ros::NodeHandle(nhp, "y"));
    pid_.z.init(ros::NodeHandle(nhp, "z"));
    pid_.yaw.init(ros::NodeHandle(nhp, "yaw"));

    center_cross_point_.setZ(waiting_height_);
    tracking_stage_ = START_POINT_STAGE;
    //tracking_stage_ = IDLE_STAGE;

    pub_twist_cmd_ = nh_.advertise<geometry_msgs::TwistStamped>(twist_cmd_topic_name_, 1);
    sub_uav_imu_ = nh_.subscribe<sensor_msgs::Imu>(uav_imu_topic_name_, 1, boost::bind(&MotionTracking::uavImuCallback, this, _1));
    sub_heliport_pos_ = nh_.subscribe<geometry_msgs::PointStamped>(heliport_pos_topic_name_, 1, boost::bind(&MotionTracking::heliportCallback, this, _1));

    /* cheat mode */
    if(cheating_mode_uav_)
      sub_uav_ground_truth_ = nh_.subscribe<geometry_msgs::PoseStamped>(uav_ground_truth_topic_name_, 1, boost::bind(&MotionTracking::uavGroundTruthCallback, this, _1)); //for uav groundtruth
    else
      sub_uav_state_ = nh_.subscribe<nav_msgs::Odometry>(uav_state_topic_name_, 1, boost::bind(&MotionTracking::uavStateCallback, this, _1));

    //if(cheating_mode_heliport_) // this will be shifted to the callback function
    sub_truck_ground_truth_ = nh_.subscribe<nav_msgs::Odometry>(truck_ground_truth_topic_name_, 1, boost::bind(&MotionTracking::truckGroundTruthCallback, this, _1)); //for truck groundtruth

    /* start arming(engaging) motor */
    motor_engage_client_ = nh_.serviceClient<std_srvs::Empty>("engage");
    motor_shutdown_client_ = nh_.serviceClient<std_srvs::Empty>("shutdown");

    tracking_start_service_srv_ =  nh_.advertiseService("start_tracking", &MotionTracking::startTrackingCallback, this);

    std_srvs::Empty srv;
    while(1)
      {
        if(motor_engage_client_.call(srv)) break;
      }

    uav_state_updated_flag_ = false;
    uav_imu_updated_flag_ = false;
    heliport_pos_updated_flag_ = false;

    tracking_thread_ = boost::thread(boost::bind(&MotionTracking::trackingFunction, this));

  }

  MotionTracking::~MotionTracking()
  {
    tracking_thread_.interrupt();
    tracking_thread_.join();
  }

  void MotionTracking::uavStateCallback(const nav_msgs::OdometryConstPtr &state)
  {
    tf::poseMsgToTF(state->pose.pose, uav_state_);
    uav_state_updated_flag_ = true;
  }

  void MotionTracking::heliportCallback(const geometry_msgs::PointStampedConstPtr &state)
  {
    heliport_pos_updated_flag_ = true;
    tf::Vector3 heliport_pos(state->point.x, state->point.y, heliport_height_); // z = 1.1, hardcoding
    tf::Quaternion heliport_orientation;
    setHeliportPose(heliport_pos, heliport_orientation);
  }

  void MotionTracking::uavImuCallback(const sensor_msgs::ImuConstPtr &imu)
  {
    tf::quaternionMsgToTF(imu->orientation, uav_imu_att_);
    tf::vector3MsgToTF(imu->linear_acceleration, uav_imu_acc_);
    uav_imu_updated_flag_ = true;
  }

  void MotionTracking::uavGroundTruthCallback(const geometry_msgs::PoseStampedConstPtr &ground_truth)
  {
    tf::poseMsgToTF(ground_truth->pose, uav_state_);
    uav_state_updated_flag_ = true;
  }

  void MotionTracking::truckGroundTruthCallback(const nav_msgs::OdometryConstPtr &ground_truth)
  {
    tf::Quaternion q(ground_truth->pose.pose.orientation.x, ground_truth->pose.pose.orientation.y, ground_truth->pose.pose.orientation.z, ground_truth->pose.pose.orientation.w);
    tf::Matrix3x3 rotation(q);
    tfScalar roll = 0, pitch = 0, yaw = 0;
    rotation.getRPY(roll, pitch, yaw);
    tf::Vector3 origin(ground_truth->pose.pose.position.x + (-0.5) * cos(yaw), ground_truth->pose.pose.position.y  + (-0.5) * sin(yaw), heliport_height_);

    /* in cheat mode we use both position and yaw data */
    if(cheating_mode_heliport_)
      {
        setHeliportPose(origin, q);
        heliport_pos_updated_flag_ = true;
      }
    else /*TODO:temporarily we use yaw data form ground truth */
      setHeliportOrientation(q);
  }

  void MotionTracking::trackingFunction()
  {
    ros::Rate loop_rate(tracking_loop_rate_);
    ros::Duration period(1 / tracking_loop_rate_);

    /* the other strategy of takeoff */
    int takeoff_cnt = tracking_loop_rate_; // 2[s]

    while(ros::ok())
      {
        /* escape if no sensor data */
        if(!uav_state_updated_flag_) continue;
        /* this scope is about the wating strategy tracking  */
        if(tracking_mode_ == WAITING_MODE)
          {
            /* this is first two stage before tracking truck */
            if(tracking_stage_ < TRUCK_TRACKING_STAGE)
              {
                tf::Vector3 feedback_pos, feedback_att;
                tf::Vector3 att_command_(0,0,0);

                /* use the twist(vel) command mode, which is nested pid: pos -> vel -> angle(acc) */
                if(control_mode_ == TWIST_MODE)
                  {
                    if(!uav_state_updated_flag_) continue;

                    feedback_pos = getUavOrigin();

                    tfScalar roll = 0, pitch = 0, yaw = 0;
                    getUavPose().getBasis().getRPY(roll, pitch, yaw);
                    feedback_att.setValue(roll, pitch, yaw);

                    /* pos control from start point to the center cross point */
                    /* this is PI control */
                    linear_command_.setX(pid_.x.update(center_cross_point_.x(), feedback_pos.x(), 0.0, period));
                    linear_command_.setY(pid_.y.update(center_cross_point_.y(), feedback_pos.y(), 0.0, period));
                    linear_command_.setZ(pid_.z.update(center_cross_point_.z(), feedback_pos.z(), 0.0, period));
                    anguler_command_.setZ(pid_.yaw.update(att_command_.z(), feedback_att.z(), period));

                    /* takeoff limitation */
                    if(feedback_pos.z() < takeoff_height_)
                      {
                        linear_command_.setX(0);
                        linear_command_.setY(0);
                        //ROS_WARN("takeoff");
                      }

                    /* limitation */
                    double vel = linear_command_.length();
                    if(vel > velocity_limit_)
                      linear_command_ *= (velocity_limit_ / vel) ;

                    geometry_msgs::TwistStamped twist_msg;
                    twist_msg.header.stamp = ros::Time::now();
                    vector3TFToMsg(linear_command_, twist_msg.twist.linear);
                    vector3TFToMsg(anguler_command_, twist_msg.twist.angular);
                    pub_twist_cmd_.publish(twist_msg);
                  }

                if(tracking_stage_ == START_POINT_STAGE )
                  {
                    tf::Vector3 pos_error = center_cross_point_ - feedback_pos;
                    pos_error.setZ(0); // we ignore the height control right now

                    /* close to the center cross point, shift CENTER_CROSS_STAGE,
                       which is aimed to wait the truck across the center poin */
                    if(pos_error.length() < center_cross_point_threshold_)
                      {
                        ROS_WARN("[UAV motion tracking]: shift to CENTER_CROSS_STAGE, pos error is %f", pos_error.length());
                        tracking_stage_ = CENTER_CROSS_STAGE;
                        continue;
                      }
                  }

                if(tracking_stage_ == CENTER_CROSS_STAGE)
                  {
                    tf::Pose heliport;
                    tf::Vector3 pos_error;
                    bool shift_stage = false;

                    if(!heliport_pos_updated_flag_) continue;

                    pos_error = center_cross_point_ - getHeliportOrigin();
                    pos_error.setZ(0);

                    if(pos_error.length() < center_cross_point_threshold_) shift_stage = true;

                    /* when the truck across the center point, then shift to TRUCK_TRACKING_STAGE */
                    if(shift_stage)
                      {
                        ROS_WARN("[UAV motion tracking]: shift to TRUCK_TRACKING_STAGE, truck yaw is %f", pos_error.length());

                        tracking_stage_ = TRUCK_TRACKING_STAGE;
                        continue;
                      }
                  }
              }

            if(tracking_stage_ == TRUCK_TRACKING_STAGE)
              {
                tf::Pose heliport;

                if(control_mode_ == TWIST_MODE)
                  {
                    tf::Vector3 feedback_pos = getUavOrigin();
                    tf::Vector3 feedback_att;
                    tf::Vector3 att_command_(0,0,0);

                    tfScalar roll = 0, pitch = 0, yaw = 0;
                    getUavPose().getBasis().getRPY(roll, pitch, yaw);
                    feedback_att.setValue(roll, pitch, yaw);

                    /* pos control from start point to the center cross point */
                    /* this is PI control */
                    tf::Vector3 heliport_origin = getHeliportOrigin();
                    linear_command_.setX(pid_.x.update(heliport_origin.x(), feedback_pos.x(), 0, period));
                    linear_command_.setY(pid_.y.update(heliport_origin.y(), feedback_pos.y(), 0, period));
                    linear_command_.setZ(pid_.z.update(heliport_origin.z(), feedback_pos.z(), 0, period));
                    if(linear_command_.z() < descending_velocity_limit_) linear_command_.setZ(descending_velocity_limit_);
                    anguler_command_.setZ(pid_.yaw.update(att_command_.z(), feedback_att.z(), period));

                    /* four pattern of truck movement => feed-forward */
#if 0
                    if(cheating_mode_heliport_)
#else
                    if(1)
#endif
                      {
                        tf::Matrix3x3 heliport_rotation(getHeliportOrientation());
                        heliport_rotation.getRPY(roll, pitch, yaw);

                        linear_command_ += tf::Vector3(cos(yaw) * truck_speed_, sin(yaw) * truck_speed_, 0 );
                      }
                    /* limitation */
                    double vel = linear_command_.length();
                    if(vel > velocity_limit_) 
                      {
                        //ROS_WARN("[UAV motion tracking]: exceeds the vel limitation: %f", vel);
                        linear_command_ *= (velocity_limit_ / vel) ;
                      }

                    /* final landing approach */
                    tf::Vector3 landing_pos(heliport_origin.x() - feedback_pos.x(), heliport_origin.y() - feedback_pos.y(), 0);
                    if(landing_pos.length() < landing_level_distance_threshold_)
                      {/* first check the horizontal pos error */

                        ROS_INFO("final landing approach, height offest is %f", feedback_pos.z()- heliport.getOrigin().z());
                        if(feedback_pos.z()- heliport_origin.z() < landing_vertical_distance_threshold_)
                          {
                            ROS_WARN("final approach, %f", feedback_pos.z()- heliport_origin.z());
#if 1
                            /* TODO:use smart landing height control!!  */
                            linear_command_.setZ(landing_vertical_velocity_);
                            if(feedback_pos.z()- heliport_origin.z() < landing_vertical_distance_threshold2_)
                              {/* shutdown motors */
                                ROS_WARN("shutdown motors");
                                std_srvs::Empty srv;
                                motor_shutdown_client_.call(srv); // no response
                                tracking_stage_ = IDLE_STAGE;
                              }
#else //just shutdown
                            /* Temporarily: shutdown motor */
                            std_srvs::Empty srv;
                            motor_shutdown_client_.call(srv); // no response
#endif
                          }
                      }

                    geometry_msgs::TwistStamped twist_msg;
                    twist_msg.header.stamp = ros::Time::now();
                    vector3TFToMsg(linear_command_, twist_msg.twist.linear);
                    vector3TFToMsg(anguler_command_, twist_msg.twist.angular);
                    pub_twist_cmd_.publish(twist_msg);
                  }
              }
          }

        ros::spinOnce();
        loop_rate.sleep();
      }
  }
} // namespace uav
