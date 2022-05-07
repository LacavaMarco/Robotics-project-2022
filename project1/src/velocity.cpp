#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <dynamic_reconfigure/server.h>
#include <project1/velParametersConfig.h>

#include <array>
#include <math.h>

class Velocity {

public:
    Velocity() {
        n.getParam("/wheels/r", r);
        n.getParam("/wheels/l", l);
        n.getParam("/wheels/w", w);
        inv_H0 = inverseH0(r, l, w);
        n.getParam("/wheels/N", cpr);
        n.getParam("/wheels/T", gear_ratio);

        last_ticks = {{0, 0, 0, 0}};
        last_time = ros::Time::now();

        sub = n.subscribe("wheel_states", 1000, &Velocity::velocityCallback, this);
        pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);

        f = boost::bind(&Velocity::param_callback, this, _1, _2);
        dynServer.setCallback(f);
    }

    // Receive the wheel_states message and for each wheel given the position in ticks,
    // compute the wheel velocity in or rad/s. Then by applying the inverse formula of
    // the kinematic model of the mobile robot with four mecanum wheels, compute robot linear
    // and angular speed and publish them on cmd_vel
    void velocityCallback(const sensor_msgs::JointState::ConstPtr& msg){
        // Ticks are stored in the array position [fl, fr, rl, rr] and they
        // increase every timestep: to compute each wheel angular velocity I need to divide the difference
        // between current and previous ticks by the difference between current and previous time stamp (?)
        ros::Time current_time = msg->header.stamp;
        double dt = (current_time - last_time).toSec();
        // ROS_INFO ("dt = %f", dt);

        std::array<int, 4> current_ticks = {{0, 0, 0, 0}};
        for(int i = 0; i < 4; i++) {
            current_ticks[i] = msg->position[i];
        }
        std::array<int, 4> dticks = {{current_ticks[0] - last_ticks[0], current_ticks[1] - last_ticks[1],
                        current_ticks[2] - last_ticks[2], current_ticks[3] - last_ticks[3]}};

        // for(int i = 0; i < 4; i++) {
        //     ROS_INFO ("dticks[%d] = %d", i, dticks[i]);
        // }

        std::array<double, 4> wheels_angular_vel = {{0.0, 0.0, 0.0, 0.0}};
        for(int i = 0; i < 4; i++) {
            wheels_angular_vel[i] = 2 * M_PI * dticks[i] / cpr / gear_ratio / dt;
            // Compare computed and actual wheels speeds (just for debugging)
            // ROS_INFO ("wheels_angular_vel[%d] = %f", i, wheels_angular_vel[i]);
            // ROS_INFO ("wheels_bag_vel[%d] = %f\n", i, msg->velocity[i] / 60 / gear_ratio);
        }
        // ROS_INFO ("\n");

        std::array<double, 3> robot_vel = computeRobotVelocity(wheels_angular_vel);

        robot_velocity.header.frame_id = "";
        robot_velocity.header.stamp = msg->header.stamp;

        robot_velocity.twist.linear = toVector3(robot_vel[1], robot_vel[2], 0);
        robot_velocity.twist.angular = toVector3(0, 0, robot_vel[0]);

        pub.publish(robot_velocity);

        last_time = current_time;
        for(int i = 0; i < 4; i++) {
            last_ticks[i] = current_ticks[i];
        }
    }

    // Compute robot linear and angular velocity from wheel angular velocities
    std::array<double, 3> computeRobotVelocity(std::array<double, 4> wheels_angular_vel) {
        std::array<double, 3> robot_vel = {{0.0, 0.0, 0.0}};
        // The data retrieved from the bag considers the wheels in the order fl, fr, rl, rr, while the
        // mecanum wheels formula used to compute the robot velocity has the wheels in the order fl, fr, rr, rl
        std::array<double, 4> wheels_ordered_vel = {{wheels_angular_vel[0], wheels_angular_vel[1],
                                                    wheels_angular_vel[3], wheels_angular_vel[2]}};

        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 4; j++) {
                robot_vel[i] += inv_H0[i][j] * wheels_ordered_vel[j];
            }
            // ROS_INFO("robot_vel[%d] = %f", i, robot_vel[i]);
        }

        return robot_vel;
    }

    // Compute the inverse of H(0) given wheels r, l and w (global parameters)
    std::array<std::array<double, 4>, 3> inverseH0(double r, double l, double w) {
        std::array<std::array<double, 4>, 3> inv_H0 = {{{-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)},
                                                        {1, 1, 1, 1},
                                                        {-1, 1, -1, 1}}};

        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 4; j++) {
                inv_H0[i][j] = inv_H0[i][j] * r / 4;
                // ROS_INFO("inv_H0[%d][%d] = %f  ", i, j, inv_H0[i][j]);
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

    // Radius and cpr dynamic_reconfigure is used only for calibration purposes:
    // notice that once that the calibrated values will be chosen, they will be
    // directly assigned in the launch file
    void param_callback(project1::velParametersConfig &config, uint32_t level) {
        // ROS_INFO("Reconfigure Request: %f, %d - Level %d", config.radius, config.cpr, level);
        r = config.radius;
        cpr = config.cpr;
        inv_H0 = inverseH0(r, l, w);
    }

private:
    ros::NodeHandle n;
    geometry_msgs::TwistStamped robot_velocity;
    ros::Subscriber sub;
    ros::Publisher pub;
    dynamic_reconfigure::Server<project1::velParametersConfig> dynServer;
    dynamic_reconfigure::Server<project1::velParametersConfig>::CallbackType f;

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
