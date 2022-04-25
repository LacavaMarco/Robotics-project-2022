#include "ros/ros.h"
#include "project1/ResetPose.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "reset_client");
    if (argc != 4) {
        ROS_INFO("argc = %d", argc);
        ROS_INFO("usage: client new_pose");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<project1::ResetPose>("reset_pose");
    project1::ResetPose srv;

    //Reset pose have argv[1] = new x, argv[2] = new y, argv[3] = new theta
    srv.request.new_x = atof(argv[1]);
    srv.request.new_y = atof(argv[2]);
    srv.request.new_theta = atof(argv[3]);

    if (client.call(srv)) {
        // ROS_INFO("Old position: (X: %f, Y: %f, Theta: %f)", (double)srv.response.old_x, (double)srv.response.old_y, (double)srv.response.old_theta);
    }
    else {
        ROS_ERROR("Failed to call service reset");
        return 1;
    }

    return 0;
}
