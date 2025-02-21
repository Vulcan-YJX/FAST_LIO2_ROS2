#include "LIVMapper.h"

int main(int argc, char **argv)
{
  // ros::init(argc, argv, "laserMapping");
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("laserMapping");
  LIVMapper mapper(nh); 
  mapper.initializeSubscribersAndPublishers(nh);
  mapper.run();
  return 0;
}