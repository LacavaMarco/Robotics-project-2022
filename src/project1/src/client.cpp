#include "ros/ros.h"
#include <project1/Reset.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "reset_client");
    if (argc != 2)
    {
      ROS_INFO("usage: client new_pose");
      return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<project1::Reset>("reset");
    project1::Reset srv;

    //Resets pose having argv[1] = new x, argv[2] = new y, argv[3] = new theta
    srv.request.new_pose.position.x = atof(argv[1]);
    srv.request.new_pose.position.y = atof(argv[2]);
    srv.request.new_pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, atof(argv[3]));
    srv.request.new_pose.orientation.x = q.x();
    srv.request.new_pose.orientation.y = q.y();
    srv.request.new_pose.orientation.z = q.z();
    srv.request.new_pose.orientation.w = q.w();

    if (client.call(srv))
    {
      ROS_INFO("Old position: X:%f Y:%f", (double)srv.response.old_pose.position.x, (double)srv.response.old_pose.position.y);
    }
    else
    {
      ROS_ERROR("Failed to call service reset");
      return 1;
    }

    return 0;
}
