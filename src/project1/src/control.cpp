#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "project1/WheelsVector4.h"

#include <array>
#include <math.h>

// To check if the results match, use rqt_plot (the bag must be played with the --clock option)
class Control {

public:
    Control() {
        n.getParam("/wheels/r", r);
        n.getParam("/wheels/l", l);
        n.getParam("/wheels/w", w);
        h0 = computeH0(r, l, w);

        sub = n.subscribe("/cmd_vel", 1000, &Control::controlCallback, this);
        pub = n.advertise<project1::WheelsVector4>("/wheels_rpm", 1000); // The actual unit of measure is rad/min
    }

    // Receive the cmd_vel message and for each wheel compute the velocity in rpm given
    // the robot linear and angular velocities by applying the formula of the kinematic
    // model of the mobile robot with four mecanum wheels and publish them on topic wheels_rpm
    void controlCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
        std::array<double, 3> robot_vel = {{msg->twist.angular.z, msg->twist.linear.x, msg->twist.linear.y}};
        std::array<double, 4> wheels_control_vel = computeWheelsVelocity(robot_vel);

        // By multiplying the robot velocities by H(0), I retrieve the wheels angular
        // velocity (rad/s): to check if the control values match the bag ones I need
        // to multiply the result by 60 (rad/min) and by the gear ratio
        for(int i = 0; i < 3; i++) {
            wheels_control_vel[i] = wheels_control_vel[i] * 60 * 5;
        }

        control_wheels_velocity.header.frame_id = "";
        control_wheels_velocity.header.stamp = msg->header.stamp;

        control_wheels_velocity.rpm_fl = wheels_control_vel[0];
        control_wheels_velocity.rpm_fr = wheels_control_vel[1];
        control_wheels_velocity.rpm_rr = wheels_control_vel[2];
        control_wheels_velocity.rpm_rl = wheels_control_vel[3];

        pub.publish(control_wheels_velocity);
    }

    // Compute wheels angular velocities from robot linear and angular velocity
    std::array<double, 4> computeWheelsVelocity(std::array<double, 3> robot_vel) {
        // The result velocities are already in the correct order to be published in wheels_rpm topic (fl, fr, rr, rl)
        std::array<double, 4> wheels_control_vel = {{0.0, 0.0, 0.0, 0.0}};

        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 3; j++) {
                wheels_control_vel[i] += h0[i][j] * robot_vel[j];
            }
            // ROS_INFO("wheels_control_vel[%d] = %f", i, wheels_control_vel[i]);
        }

        return wheels_control_vel;
    }

    // Compute the matrix H(0) given wheels r, l and w (global parameters)
    std::array<std::array<double, 3>, 4> computeH0(double r, double l, double w) {
        std::array<std::array<double, 3>, 4> h0 = {{{-l-w, 1, -1},
                                                    {l+w, 1, 1},
                                                    {l+w, 1, -1},
                                                    {-l-w, 1, 1}}};

        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 3; j++) {
                h0[i][j] = h0[i][j] / r;
                // ROS_INFO("H0[%d][%d] = %f  ", i, j, h0[i][j]);
            }
        }
        return h0;
    }

private:
    ros::NodeHandle n;
    project1::WheelsVector4 control_wheels_velocity;
    ros::Subscriber sub;
    ros::Publisher pub;

    double r;
    double l;
    double w;
    std::array<std::array<double, 3>, 4> h0;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "control_evaluator");
    Control control;
    ros::spin();
    return 0;
}
