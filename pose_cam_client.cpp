#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "chapter2_tutorials/PoseCam.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_cam_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<chapter2_tutorials::PoseCam>("pose_cam");
  chapter2_tutorials::PoseCam srv;
  srv.request.solicitud = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("X: %f", (float)srv.response.PositionCap.x);
    ROS_INFO("Y: %f", (float)srv.response.PositionCap.y);
    ROS_INFO("Z: %f", (float)srv.response.PositionCap.z);
    ROS_INFO("XB: %f", (float)srv.response.PositionBottle.x);
    ROS_INFO("YB: %f", (float)srv.response.PositionBottle.y);
    ROS_INFO("ZB: %f", (float)srv.response.PositionBottle.z);
    ROS_INFO("Confirmacion: %d", (long int)srv.response.ok);

    float xc=srv.response.PositionCap.x;
    float yc=srv.response.PositionCap.y;
    float zc=srv.response.PositionCap.z;
    float xb=srv.response.PositionBottle.x;
    float yb=srv.response.PositionBottle.y;
    float zb=srv.response.PositionBottle.z;
    int confirmacion=srv.response.ok;

    printf("PoseCap: %f,%f,%f\n",xc,yc,zc);
    printf("PoseBot: %f,%f,%f\n",xb,yb,zb);
    printf("Confirmacion: %i",confirmacion);


  }
  else
  {
    ROS_ERROR("Failed to call service pose_cam");
    return 1;
  }

  return 0;
}
