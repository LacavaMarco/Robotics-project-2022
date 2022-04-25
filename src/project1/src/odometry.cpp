#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <dynamic_reconfigure/server.h>
#include <project1/odomParametersConfig.h>
#include "project1/ResetPose.h"

#include <array>

class Odometry {

public:
    Odometry() {
        last_time = ros::Time::now();
        int_method = 0;
        getInitialPose = true;
        resetInitialPose = true;

        f = boost::bind(&Odometry::param_callback, this, _1, _2);
        dynServer.setCallback(f);

        server = n.advertiseService<project1::ResetPose::Request,
                                    project1::ResetPose::Response>("reset_pose",
                                                                    boost::bind(&Odometry::reset_callback,
                                                                    this, _1, _2));

        sub_v = n.subscribe("/cmd_vel", 1000, &Odometry::odometryCallback, this);
        pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);
    }

    // Initialize pose with the values of the first message of /robot/pose topic
    // Could be used also to reset the pose when implementing the service (?)
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
        // ROS_INFO("initial pose: (%f, %f, %f)", curr_x, curr_y, curr_theta);
    }

    // Receive the robot linear and angular velocities and compute the robot odometry by
    // applying Euler or Runge-Kutta integration method (chosen using dynamic reconfigure),
    // then publish the odometry on topic /odom
    void odometryCallback(const geometry_msgs::TwistStamped::ConstPtr& msg_v) {
        if(getInitialPose) {
            // Wait for the first message of topic /robot/pose to initialize the pose of the robot
            if(resetInitialPose) {
                geometry_msgs::PoseStamped::ConstPtr initial_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/robot/pose", n);
                initializePose(initial_pose);
            }
            last_time = msg_v->header.stamp; // first dt will be 0 (minor inconvenince)
            getInitialPose = false;
            resetInitialPose = true;
        }

        ros::Time current_time = msg_v->header.stamp;
        robot_odometry.header.frame_id = "odom";
        robot_odometry.header.stamp = msg_v->header.stamp;

        // ROS_INFO("Chosen method: %d", int_method==0 ? "Euler" : "Runge-Kutta");
        robot_odometry.pose.pose = computeOdometry(msg_v->twist.linear,
                                                    msg_v->twist.angular,
                                                    current_time,
                                                    last_time,
                                                    int_method);

        robot_odometry.twist.twist.linear = msg_v->twist.linear;
        robot_odometry.twist.twist.angular = msg_v->twist.angular;

        pub.publish(robot_odometry);

        last_time = current_time;
    }

    // Odometry integration method
    geometry_msgs::Pose computeOdometry(geometry_msgs::Vector3 vel_lin,
                                            geometry_msgs::Vector3 vel_ang,
                                            ros::Time current_time,
                                            ros::Time last_time,
                                            int integration_method) {

        auto odom_pos = geometry_msgs::Pose();

        double dt = (current_time - last_time).toSec(); //sampling period t
        // ROS_INFO("dt = %f", dt);

        if(integration_method == 0) { // Euler
            curr_x = curr_x + (vel_lin.x*cos(curr_theta) + vel_lin.y*cos(M_PI/2 + curr_theta)) * dt;
            curr_y = curr_y + (vel_lin.x*sin(curr_theta) + vel_lin.y*sin(M_PI/2 + curr_theta)) * dt;
        }
        else { // Runge-Kutta
            curr_x = curr_x + (vel_lin.x*cos(curr_theta + vel_ang.z*dt/2) + vel_lin.y*cos(M_PI/2 + curr_theta + vel_ang.z*dt/2)) * dt;
            curr_y = curr_y + (vel_lin.x*sin(curr_theta + vel_ang.z*dt/2) + vel_lin.y*sin(M_PI/2 + curr_theta + vel_ang.z*dt/2)) * dt;
        }

        odom_pos.position.x = curr_x;
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

    void param_callback(project1::odomParametersConfig &config, uint32_t level) {
        // ROS_INFO("Reconfigure Request: %s, %d - Level %d", config.getInitialPose? "True" : "False", config.int_method, level);
        // Before playing another bag, in order to let the initial robot
        // pose be read, I need to set getInitialPose back to true
        getInitialPose = config.getInitialPose;
        int_method = config.int_method;
    }

    bool reset_callback(project1::ResetPose::Request &req, project1::ResetPose::Response &res){
        res.old_x = robot_odometry.pose.pose.position.x;
        res.old_y = robot_odometry.pose.pose.position.y;

        tf2::Quaternion q(
            robot_odometry.pose.pose.orientation.x,
            robot_odometry.pose.pose.orientation.y,
            robot_odometry.pose.pose.orientation.z,
            robot_odometry.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        res.old_theta = yaw;

        curr_x = req.new_x;
        curr_y = req.new_y;
        curr_theta = req.new_theta;

        robot_odometry.header.frame_id = "odom";
        robot_odometry.header.stamp = last_time;

        robot_odometry.pose.pose.position.x = curr_x;
        robot_odometry.pose.pose.position.y = curr_y;
        robot_odometry.pose.pose.position.z = 0;

        tf2::Quaternion p;
        p.setRPY(0, 0, curr_theta);
        robot_odometry.pose.pose.orientation.x = p.x();
        robot_odometry.pose.pose.orientation.y = p.y();
        robot_odometry.pose.pose.orientation.z = p.z();
        robot_odometry.pose.pose.orientation.w = p.w();

        pub.publish(robot_odometry);

        // If reset_pose service is called before the bag is played,
        // the initial pose of the robot must not be updated
        resetInitialPose = false;

        // ROS_INFO("Reset pose from (X: %f, Y: %f, Theta: %f) to (X: %f, Y: %f, Theta: %f)",
        //             (double) req.new_x, (double) req.new_y, (double) req.new_theta,
        //             (double) res.old_x, (double) res.old_y, (double) res.old_theta);
        return true;
    }

private:
    ros::NodeHandle n;
    nav_msgs::Odometry robot_odometry;
    ros::Subscriber sub_p;
    ros::Subscriber sub_v;
    ros::Publisher pub;

    dynamic_reconfigure::Server<project1::odomParametersConfig> dynServer;
    dynamic_reconfigure::Server<project1::odomParametersConfig>::CallbackType f;

    ros::ServiceServer server;

    int int_method;
    ros::Time last_time;
    bool getInitialPose;
    // true if the service /reset_pose is called before a bag is played (no need to update the initial pose again)
    bool resetInitialPose;

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
