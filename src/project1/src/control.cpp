#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <array>
#include <math.h>

class Control
{
public:
    Control() {
        n.getParam("/wheels/r", r); // probably will need 4 different radius instances for each wheel
        n.getParam("/wheels/l", l);
        n.getParam("/wheels/w", w);
        h0 = h0(r, l, w);
        n.getParam("/wheels/N", cpr);
        n.getParam("/wheels/T", gear_ratio);

        last_ticks = {{0, 0, 0, 0}};
        last_time = ros::Time::now();

        sub = n.subscribe("/cmd_vel", 1000, &Control::controlCallback, this);
        pub = n.advertise<???>("wheels_rpm", 1000);
    }

    // Receive the cmd_vel message and for each wheel compute the velocity in rpm given
    // the robot linear and angular velocities by applying the formula of the kinematic
    // model of the mobile robot with four mecanum wheels and publish them on wheels_rpm
    void controlCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
        // By multiplying the robot velocities by H(0), I retrieve the wheels angular
        // velocity (rad/s): to get it in rpm I need to multiply the result by 60/2pi

        std::array<double, 3> robot_vel = {{velocity->twist.linear.x, velocity->twist.linear.y, velocity->twist.angular.z}}

        std::array<double, 4> wheels_rpm = computeWheelsRPM(robot_vel);

        auto rpm_msg = ???();
        vel_msg.header.frame_id = "";
        vel_msg.header.stamp = ros::Time::now();

        vel_msg.twist.linear = toVector3(robot_vel[1], robot_vel[2], 0);

        pub.publish(vel_msg);

    }

    // Compute robot linear and angular velocity from wheel angular velocities
    std::array<double, 4> computeWheelsRPM(std::array<double, 3> robot_vel) {
        // The result velocities are already in the correct order to be published in wheels_rpm topic (fl, fr, rr, rl)
        std::array<double, 4> wheels_rpm = {{0.0, 0.0, 0.0, 0.0}};

        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 3; j++) {
                wheels_rpm[i] += h0[i][j] * robot_vel[j];
            }
            ROS_INFO("wheels_rpm[%d] = %f", i, wheels_rpm[i]);
        }

        return wheels_rpm;
    }

    // Compute the inverse of H(0) given wheels r, l and w (global parameters)
    std::array<std::array<double, 3>, 4> h0(double r, double l, double w) {
        std::array<std::array<double, 3>, 4> h0 = {{}};

        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 4; j++) {
                h0[i][j] = h0[i][j] / r;
                // ROS_INFO("H0[%d][%d] = %f  ", i, j, h0[i][j]);
            }
        }
        return inv_H0;
    }

    // Convert 3 double variables into Vector3 (used in geometry_msgs::TwistStamped)
    geometry_msgs::Vector3 toVector3(double x, double y, double z) {
    	auto v3 = geometry_msgs::Vector3();
    	v3.x = x;
    	v3.y = y;
    	v3.z = z;

    	return v3;
    }

private:
    ros::NodeHandle n;
    geometry_msgs::TwistStamped robot_velocity;
    ros::Subscriber sub;
    ros::Publisher pub;

    double r;
    double l;
    double w;
    std::array<std::array<double, 4>, 3> inv_H0;
    int cpr;
    int gear_ratio;

    std::array<int, 4> last_ticks;
    ros::Time last_time;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_estimator");
    Velocity velocity;
    ros::spin();
    return 0;
}
