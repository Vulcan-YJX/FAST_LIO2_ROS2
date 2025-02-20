#include "LIVMapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  LIVMapper mapper(nh); 
  mapper.initializeSubscribersAndPublishers(nh);
  mapper.run();
  return 0;
}