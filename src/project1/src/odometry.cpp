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
        // sub_p = n.subscribe("/robot/pose", 1000, &Odometry::poseCallback, this);

        // Wait for the first message of topic /robot/pose to initialize the pose of the robot
        geometry_msgs::PoseStamped::ConstPtr initial_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/robot/pose", n);
        initializePose(initial_pose);

        sub_v = n.subscribe("/cmd_vel", 1000, &Odometry::odometryCallback, this);
        pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);
    }

    void initializePose(const geometry_msgs::PoseStamped::ConstPtr& msg_p){
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

    void odometryCallback(const geometry_msgs::TwistStamped::ConstPtr& msg_v) {
        ros::Time current_time = msg_v->header.stamp;
        //get dei parametri della pose attuale,
        //che ho settato con l'odometryCallback di prima.
        robot_odometry.header.frame_id = "odom";
        robot_odometry.header.stamp = msg_v->header.stamp;

        //If the current time is not the first (so if it is not 0), I compute the
        //odometry. If it is the first then I do nothing and I store the value in
        //a last_time variable, which contains the previous time and is updated at
        //the end of every cycle.

        robot_odometry.pose.pose = computeEulerOdometry(msg_v->twist.linear,
                                                msg_v->twist.angular,
                                                current_time,
                                                last_time);

        robot_odometry.twist.twist.linear = msg_v->twist.linear;
        robot_odometry.twist.twist.angular = msg_v->twist.angular;

        pub.publish(robot_odometry);

        last_time = current_time;
    }

    // Basic Euler odometry. Needs to be updated once I learn how to use quaternions.
    geometry_msgs::Pose computeEulerOdometry(geometry_msgs::Vector3 vel_lin,
                                            geometry_msgs::Vector3 vel_ang,
                                            ros::Time current_time,
                                            ros::Time last_time) {

        auto odom_pos = geometry_msgs::Pose();

        double dt = (current_time - last_time).toSec(); //sampling period t

        curr_x = curr_x + (vel_lin.x*cos(curr_theta) + vel_lin.y*cos(M_PI/2 + curr_theta)) * dt;
        odom_pos.position.x = curr_x;
        curr_y = curr_y + (vel_lin.x*sin(curr_theta) + vel_lin.y*sin(M_PI/2 + curr_theta)) * dt;
        odom_pos.position.y = curr_y;
        odom_pos.position.z = 0; // always 0, the robot moves on a plane

        curr_theta = curr_theta + vel_ang.z * dt;

        tf2::Quaternion q;
        q.setRPY(0, 0, curr_theta);
        odom_pos.orientation.x = q.x();
        odom_pos.orientation.y = q.y();
        odom_pos.orientation.z = q.z();
        odom_pos.orientation.w = q.w();

        // ROS_INFO("updated pose: (%f, %f, %f)", curr_x, curr_y, curr_theta);

        return odom_pos;
    }

private:
    ros::NodeHandle n;
    nav_msgs::Odometry robot_odometry;
    ros::Subscriber sub_p;
    ros::Subscriber sub_v;
    ros::Publisher pub;

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
