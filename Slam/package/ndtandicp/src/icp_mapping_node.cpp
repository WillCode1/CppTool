#include "icp_mapping.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "icp_mapping");
  icp_mapping icp;

  ros::spin();

  return 0;
};

