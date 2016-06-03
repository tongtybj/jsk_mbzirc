#include <jsk_mbzirc_tasks/task1_motion_tracking.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "task1_motion_tracking");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  uav::MotionTracking*  MotionTrackingNode = new uav::MotionTracking(nh, nh_private);
  ros::spin ();
  delete MotionTrackingNode;
  return 0;
}
