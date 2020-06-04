#include "ros/ros.h"
//#include "beginner_tutorials/AddTwoInts.h"
#include "sim_img_proc/get_coordinates.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_coordinates");
  /*if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }*/

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<sim_img_proc::get_coordinates>("get_coordinates");
  sim_img_proc::get_coordinates srv;
  srv.request.dummy = atoll(argv[1]);
  if (client.call(srv))
  { for(int i=0;i<sizeof(srv.response.x);i++)
    ROS_INFO("x: %ld, ", (long int)srv.response.x[i]);
    for(int i=0;i<sizeof(srv.response.y);i++)
    ROS_INFO("y: %ld, ", (long int)srv.response.y[i]);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
