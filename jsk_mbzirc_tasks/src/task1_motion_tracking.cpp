#include <jsk_mbzirc_tasks/task1_motion_tracking.h>

namespace uav
{

MotionTracking::MotionTracking(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp)
{
  nhp_.param("tracking_mode", tracking_mode_, 0);
  nhp_.param("control_mode", control_mode_, 0);
  nhp_.param("cheating_mode", cheating_mode_, true);

  nhp_.param("waiting_height", waiting_height_, 8.0); // 8m
  nhp_.param("takeoff_height", takeoff_height_, 0.0); // 8m
  nhp_.param("heliport_height_offset", heliport_height_offset_, 0.0); // 8m

  nhp_.param("truck_speed", truck_speed_, 4.0); // 15Km/h = 4.16666666m/s
  nhp_.param("velocity_limit", velocity_limit_, 8.0); // 30Km/h = 8.33333m/s
  nhp_.param("tracking_loop_rate", tracking_loop_rate_, 50.0);
  nhp_.param("center_cross_point_threshold", center_cross_point_threshold_, 1.0);

  nhp_.param("twist_cmd_topic_name", twist_cmd_topic_name_, std::string("command/twist"));
  nhp_.param("state_topic_name", state_topic_name_, std::string("state"));
  nhp_.param("imu_topic_name", imu_topic_name_, std::string("imu"));

  nhp_.param("landing_level_distance_threshold", landing_level_distance_threshold_, 0.6);
  nhp_.param("landing_vertical_distance_threshold", landing_vertical_distance_threshold_, 0.2);
  nhp_.param("landing_vertical_distance_threshold2", landing_vertical_distance_threshold2_, 0.05);
  nhp_.param("landing_vertical_velocity", landing_vertical_velocity_, -0.5);

  /* cheating mode */
  nhp_.param("ground_truth_topic_name", ground_truth_topic_name_, std::string("ground_truth_to_tf/pose"));

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

  pub_twist_cmd_ = nh_.advertise<geometry_msgs::TwistStamped>(twist_cmd_topic_name_, 1);
  sub_estimated_state_ = nh_.subscribe<nav_msgs::Odometry>(state_topic_name_, 1, boost::bind(&MotionTracking::stateCallback, this, _1));
  sub_imu_ = nh_.subscribe<sensor_msgs::Imu>(imu_topic_name_, 1, boost::bind(&MotionTracking::imuCallback, this, _1));
  /* cheat mode */
  sub_ground_truth_ = nh_.subscribe<geometry_msgs::PoseStamped>(ground_truth_topic_name_, 1, boost::bind(&MotionTracking::groundTruthCallback, this, _1));

  /* start arming(engaging) motor */
  motor_engage_client_ = nh_.serviceClient<std_srvs::Empty>("engage");
  motor_shutdown_client_ = nh_.serviceClient<std_srvs::Empty>("shutdown");

  std_srvs::Empty srv;
  while(1)
    {
      if(motor_engage_client_.call(srv)) break;
    }

  state_updated_flag_ = false;
  imu_updated_flag_ = false;

  tracking_thread_ = boost::thread(boost::bind(&MotionTracking::trackingFunction, this));

  if(cheating_mode_)
    {
      nhp_.param("tf_loop_rate", tf_loop_rate_, 50.0);

      ground_truth_updated_flag_ = false;
      tf_updated_flag_ = false;

      tf_thread_ = boost::thread(boost::bind(&MotionTracking::tfFunction, this));
    }

}

MotionTracking::~MotionTracking()
{
  if(cheating_mode_)
    {
      tf_thread_.interrupt();
      tf_thread_.join();
    }

  tracking_thread_.interrupt();
  tracking_thread_.join();
}


void MotionTracking::stateCallback(const nav_msgs::OdometryConstPtr &state)
{
  tf::poseMsgToTF(state->pose.pose, estimated_state_);
  state_updated_flag_ = true;
}

void MotionTracking::imuCallback(const sensor_msgs::ImuConstPtr &imu)
{
  tf::quaternionMsgToTF(imu->orientation, imu_att_);
  tf::vector3MsgToTF(imu->linear_acceleration, imu_acc_);
  imu_updated_flag_ = true;
}

void MotionTracking::groundTruthCallback(const geometry_msgs::PoseStampedConstPtr &ground_truth)
{
  tf::poseMsgToTF(ground_truth->pose, ground_truth_state_);
  ground_truth_updated_flag_ = true;
}

void MotionTracking::tfFunction()
{
  /* Cheat mode: get the pose of truck / heliport */
  ros::Rate loop_rate(tf_loop_rate_);
  ros::Duration dur (1 / tracking_loop_rate_);
  while(ros::ok())
    {
      if (tf_.waitForTransform("/world", "/heliport", ros::Time(0),dur))
        {
          tf::StampedTransform transform;
          try
            {
              tf_.lookupTransform("/world", "/heliport", ros::Time(0), transform);
              setGroundTruthHeliportPose(transform);
              tf_updated_flag_ = true;
              //ROS_INFO("[UAV motoin tracking]: heliport x: %f, y: %f", transform.getOrigin().x(), transform.getOrigin().y());
            }
          catch (tf::TransformException ex)
            {
              ROS_ERROR("%s",ex.what());
            }
        }
      loop_rate.sleep();
    }
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
      if(!state_updated_flag_) continue;

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
                  if(cheating_mode_)
                    {
                      if(!ground_truth_updated_flag_) continue;

                      feedback_pos.setValue(ground_truth_state_.getOrigin().x(),
                                            ground_truth_state_.getOrigin().y(),
                                            ground_truth_state_.getOrigin().z());
                      tfScalar roll = 0, pitch = 0, yaw = 0;
                      ground_truth_state_.getBasis().getRPY(roll, pitch, yaw);
                      feedback_att.setValue(roll, pitch, yaw);
                    }
                  else
                    {
                      /* TODO: use the gps based position for the feeback, two step: */
                      /* 1) Change from WGS84 to NEU coord, 2) change to arena coord */
                      /* should also set the yaw command */
                    }

                  /* pos control from start point to the center cross point */
                  /* this is PI control */
                  linear_command_.setX(pid_.x.update(center_cross_point_.x(), feedback_pos.x(), 0.0, period));
                  linear_command_.setY(pid_.y.update(center_cross_point_.y(), feedback_pos.y(), 0.0, period));
                  linear_command_.setZ(pid_.z.update(center_cross_point_.z(), feedback_pos.z(), 0.0, period));
                  anguler_command_.setZ(pid_.yaw.update(att_command_.z(), feedback_att.z(), period));

                  /* takeoff limitation */
#if 1
                  if(feedback_pos.z() < takeoff_height_)
                    {
                      linear_command_.setX(0);
                      linear_command_.setY(0);
                      ROS_WARN("takeoff");
                    }
#else 
                  if(takeoff_cnt-- > 0)
                    {
                      linear_command_.setX(0);
                      linear_command_.setY(0);
                      ROS_WARN("takeoff");
                    }
#endif

                  /* limitation */
                  double vel = linear_command_.length();
                  if(vel > velocity_limit_)
                    {
                      //ROS_WARN("[UAV motion tracking]: exceeds the vel limitation: %f", vel);
                      linear_command_ *= (velocity_limit_ / vel) ;
                    }
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

                  if(cheating_mode_)
                    {
                      if(!tf_updated_flag_) continue;

                      heliport.setOrigin(getGroundTruthHeliportOrigin());
                      //heliport.setOrientation(getGroundTruthHeliportOrientation());
                      pos_error = center_cross_point_ - heliport.getOrigin();
                      pos_error.setZ(0);

                      if(pos_error.length() < center_cross_point_threshold_) shift_stage = true;
                    }
                  else
                    {
                      /* TODO: subscribe the landmark(truck heliport), and check the motion of truck
                       for instance, check whether truch is across the center point */
                    }

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
              double truck_yaw;

              if(control_mode_ == TWIST_MODE)
                {
                  tf::Vector3 feedback_pos, feedback_att;
                  tf::Vector3 att_command_(0,0,0);
                  if(cheating_mode_)
                    {
                      heliport.setOrigin(getGroundTruthHeliportOrigin());
                      heliport.setRotation(getGroundTruthHeliportOrientation());
                      truck_yaw= getYaw(heliport.getRotation());

                      feedback_pos.setValue(ground_truth_state_.getOrigin().x(),
                                            ground_truth_state_.getOrigin().y(),
                                            ground_truth_state_.getOrigin().z());
                      tfScalar roll = 0, pitch = 0, yaw = 0;
                      ground_truth_state_.getBasis().getRPY(roll, pitch, yaw);
                      feedback_att.setValue(roll, pitch, yaw);
                    }
                  else
                    {
                      /* TODO: use the gps based position for the feeback, two step: */
                      /* 1) Change from WGS84 to NEU coord, 2) change to arena coord */
                      /* should also set the yaw command */
                    }

                  /* pos control from start point to the center cross point */
                  /* this is PI control */
                  linear_command_.setX(pid_.x.update(heliport.getOrigin().x(), feedback_pos.x(), 0, period));
                  linear_command_.setY(pid_.y.update(heliport.getOrigin().y(), feedback_pos.y(), 0, period));
                  linear_command_.setZ(pid_.z.update(heliport.getOrigin().z() + heliport_height_offset_, feedback_pos.z(), 0, period));
                  anguler_command_.setZ(pid_.yaw.update(att_command_.z(), feedback_att.z(), period));

                  /* four pattern of truck movement => feed-forward */
                  linear_command_ += tf::Vector3(cos(truck_yaw) * truck_speed_, sin(truck_yaw) * truck_speed_, 0 );
                  //ROS_INFO("[UAV motion tracking]: feed-forwad: x: %f, y: %f", cos(truck_yaw) * truck_speed_, sin(truck_yaw) * truck_speed_);
                  //ROS_INFO("[UAV motion tracking]: com_vel_x: %f, com_vel_y: %f", linear_command_.x(), linear_command_.y());

                  /* limitation */
                  double vel = linear_command_.length();
                  if(vel > velocity_limit_) 
                    {
                      //ROS_WARN("[UAV motion tracking]: exceeds the vel limitation: %f", vel);
                      linear_command_ *= (velocity_limit_ / vel) ;
                    }


                  /* final landing approach */
                  tf::Vector3 landing_pos(heliport.getOrigin().x() - feedback_pos.x(), heliport.getOrigin().y() - feedback_pos.y(), 0);
                  if(landing_pos.length() < landing_level_distance_threshold_)
                    {/* first check the horizontal pos error */
                      ROS_INFO("final landing approach, height offest is %f", feedback_pos.z()- heliport.getOrigin().z());
                      if(feedback_pos.z()- heliport.getOrigin().z() < landing_vertical_distance_threshold_)
                        {
                          ROS_WARN("final approach, %f", feedback_pos.z()- heliport.getOrigin().z());
#if 1
                          /* TODO:use smart landing height control!!  */
                          linear_command_.setZ(landing_vertical_velocity_);
                          if(feedback_pos.z()- heliport.getOrigin().z() < landing_vertical_distance_threshold2_)
                            {/* shutdown motors */
                              ROS_WARN("shutdown motors");
                              std_srvs::Empty srv;
                              motor_shutdown_client_.call(srv); // no response
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

void MotionTracking::setGroundTruthHeliportPose(tf::StampedTransform transform)
{
  boost::mutex::scoped_lock lock(tf_mutex_);
  heliport_state_.setOrigin(transform.getOrigin());
  heliport_state_.setRotation(transform.getRotation());
}

tf::Quaternion MotionTracking::getGroundTruthHeliportOrientation()
{
  boost::mutex::scoped_lock lock(tf_mutex_);
  return heliport_state_.getRotation();
}

tf::Vector3 MotionTracking::getGroundTruthHeliportOrigin()
{
  boost::mutex::scoped_lock lock(tf_mutex_);
  return heliport_state_.getOrigin();
}

} // namespace uav
