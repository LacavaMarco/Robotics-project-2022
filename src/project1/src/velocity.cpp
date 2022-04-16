#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <array>
#include <math.h>

class Velocity
{
public:
    Velocity() {
        n.getParam("/wheels/r", r); // probably will need 4 different radius instances for each wheel
        n.getParam("/wheels/l", l);
        n.getParam("/wheels/w", w);
        inv_H0 = inverseH0(r, l, w);
        n.getParam("/wheels/N", cpr);
        n.getParam("/wheels/T", gear_ratio);

        last_ticks = {{0, 0, 0, 0}};
        last_time = ros::Time::now();

        sub = n.subscribe("/wheel_states", 1000, &Velocity::velocityCallback, this);
        pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
    }

    // Receive the wheel_states message and for each wheel given the position in ticks,
    // retrieve the wheel velocities in m/s (or rad/s). Then by applying the inverse formula of
    // the kinematic model of the mobile robot with four mecanum wheels, compute robot linear
    // and angular speed and publish them on cmd_vel by using pub.publish()
    void velocityCallback(const sensor_msgs::JointState::ConstPtr& msg){
        // Ticks are stored in the array position [fl, fr, rl, rr] and they incrementally
        // increase every timestep: to compute each wheel angular velocity I need to divide the difference
        // between current and previous ticks by the difference between current and previous time stamp (?)
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        ROS_INFO ("dt = %f", dt);

        std::array<int, 4> current_ticks = {{0, 0, 0, 0}};
        for(int i = 0; i < 4; i++) {
            current_ticks[i] = msg->position[i];
        }
        std::array<int, 4> dticks = {{current_ticks[0] - last_ticks[0], current_ticks[1] - last_ticks[1],
                        current_ticks[2] - last_ticks[2], current_ticks[3] - last_ticks[3]}};

        for(int i = 0; i < 4; i++) {
            ROS_INFO ("dticks[%d] = %d", i, dticks[i]);
        }

        // ROS_INFO ("cpr = %d", cpr);
        // ROS_INFO ("gear_ratio = %d", gear_ratio);

        std::array<double, 4> wheels_angular_vel = {{0.0, 0.0, 0.0, 0.0}};
        for(int i = 0; i < 4; i++) {
            wheels_angular_vel[i] = 2 * M_PI * dticks[i] / cpr / gear_ratio / dt;
            ROS_INFO ("wheels_angular_vel[%d] = %f", i, wheels_angular_vel[i]);
        }

        std::array<double, 3> robot_vel = computeRobotVelocity(wheels_angular_vel);

        auto vel_msg = geometry_msgs::TwistStamped();
        vel_msg.header.frame_id = "";
        vel_msg.header.stamp = ros::Time::now();

        vel_msg.twist.linear = toVector3(robot_vel[1], robot_vel[2], 0);
        vel_msg.twist.angular = toVector3(0, 0, robot_vel[0]);

        pub.publish(vel_msg);

        last_time = current_time;
        for(int i = 0; i < 4; i++) {
            last_ticks[i] = current_ticks[i];
        }
    }

    // Compute robot linear and angular velocity from wheel angular velocities
    std::array<double, 3> computeRobotVelocity(std::array<double, 4> wheels_angular_vel) {
        std::array<double, 3> robot_vel = {{0.0, 0.0, 0.0}};
        // The data retrieved from the bag has the wheels in the order fl, fr, rl, rr, while the
        // mecanum wheels formula used to compute the robot velocity has the wheels in the order fl, fr, rr, rl
        std::array<double, 4> wheels_ordered_vel = {{wheels_angular_vel[0], wheels_angular_vel[1],
                                                    wheels_angular_vel[3], wheels_angular_vel[2]}};

        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 4; j++) {
                robot_vel[i] += inv_H0[i][j] * wheels_ordered_vel[j];
            }
            ROS_INFO("robot_vel[%d] = %f", i, robot_vel[i]);
        }

        return robot_vel;
    }

    // Compute the inverse of H(0) given wheels r, l and w (global parameters)
    std::array<std::array<double, 4>, 3> inverseH0(double r, double l, double w) {
        std::array<std::array<double, 4>, 3> inv_H0 = {{{-0.25/(l+w), 0.25/(l+w), 0.25/(l+w), -0.25/(l+w)},
                                {0.25, 0.25, 0.25, 0.25},
                                {-0.25, -0.25, -0.25, -0.25}}};

        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 4; j++) {
                inv_H0[i][j] = r * inv_H0[i][j];
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
