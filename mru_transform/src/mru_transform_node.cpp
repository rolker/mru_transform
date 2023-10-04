
#include <ros/ros.h>
#include <mru_transform/mru_transform.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mru_transform");

  ros::NodeHandle nh;  
  ros::NodeHandle nh_private("~");
  
  mru_transform::MRUTransform t(nh, nh_private);
  
  ros::spin();
  return 0;
}
