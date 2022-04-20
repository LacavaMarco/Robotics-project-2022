#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <array>

class Odometry {

public:
    Odometry() { //variables in which I have to save initialpose
        last_time = ros::Time::now();
        isFirstPose = false;
        sub_p = n.subscribe("/robot/pose", 1000, &Odometry::poseCallback, this);
        sub_v = n.subscribe("/cmd_vel", 1000, &Odometry::odometryCallback, this);
        pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg_p){
        if(!isFirstPose){
            curr_x = msg_p->pose.position.x;
            curr_y = msg_p->pose.position.y;

            tf2::Quaternion q(
                msg_p->pose.orientation.x,
                msg_p->pose.orientation.y,
                msg_p->pose.orientation.z,
                msg_p->pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            curr_theta = yaw;
            isFirstPose = true;
            // ROS_INFO("initial pose: (%f, %f, %f)", curr_x, curr_y, curr_theta);
        }
    }

    void odometryCallback(const geometry_msgs::TwistStamped::ConstPtr& msg_v) {

        ros::Time current_time = ros::Time::now();
        //get dei parametri della pose attuale,
        //che ho settato con l'odometryCallback di prima.
        auto odom_msg = nav_msgs::Odometry();
        odom_msg.header.frame_id = "";
        odom_msg.header.stamp = ros::Time::now();

        //If the current time is not the first (so if it is not 0), I compute the
        //odometry. If it is the first then I do nothing and I store the value in
        //a last_time variable, which contains the previous time and is updated at
        //the end of every cycle.

        odom_msg.pose.pose = computeEulerOdometry(msg_v->twist.linear,
                                                msg_v->twist.angular,
                                                current_time,
                                                last_time,
                                                curr_x, curr_y, curr_theta);

        odom_msg.twist.twist.linear = msg_v->twist.linear;
        odom_msg.twist.twist.angular = msg_v->twist.angular;

        pub.publish(odom_msg);

        last_time = current_time;
    }

    // Basic Euler odometry. Needs to be updated once I learn how to use quaternions.
    geometry_msgs::Pose computeEulerOdometry(geometry_msgs::Vector3 vel_lin,
                                            geometry_msgs::Vector3 vel_ang,
                                            ros::Time current_time,
                                            ros::Time last_time,
                                            double x, double y, double theta) {

        auto odom_pos = geometry_msgs::Pose();

        double dt = (current_time - last_time).toSec(); //sampling period t

        odom_pos.position.x =  x + vel_lin.x * dt;
        odom_pos.position.y =  y + vel_lin.y * dt;
        odom_pos.position.z = 0; //it's alwyas 0, it moves on a plane

        curr_theta = theta + vel_ang.z * dt;

        tf2::Quaternion q;
        q.setRPY(0, 0, curr_theta);
        odom_pos.orientation.x = q.x();
        odom_pos.orientation.y = q.y();
        odom_pos.orientation.z = q.z();
        odom_pos.orientation.w = q.w();

        curr_x = odom_pos.position.x;
        curr_y = odom_pos.position.y;

        return odom_pos;
    }

private:
    ros::NodeHandle n;
    // nav_msgs::Odometry robot_odometry;
    ros::Subscriber sub_p;
    ros::Subscriber sub_v;
    ros::Publisher pub;

    bool isFirstPose;
    ros::Time last_time;

    double curr_x;
    double curr_y;
    double curr_theta;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_calculator");
    Odometry odometry;
    ros::spin();
    return 0;
}
