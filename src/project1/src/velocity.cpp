#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped>

class Velocity
{
public:
    Velocity() {
        n.getParam("/wheels/r", r); // probably will need 4 different radius instances for each wheel
        n.getParam("/wheels/l", l);
        n.getParam("/wheels/w", w);
        inv_H0 = inverseH0(r, l, w);
        sub = n.subscribe("/wheel_states", 1000, &Velocity::velocityCallback, this);
        pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
    }

    // Receive the wheel_states message and for each wheel given the position in ticks,
    // retrieve the wheel velocities in m/s (or rad/s). Then by applying the inverse formula of
    // the kinematic model of the mobile robot with four mecanum wheels, compute robot linear
    // and angular speed and publish them on cmd_vel by using pub.publish()
    void velocityCallback(const sensor_msgs::JointState::ConstPtr& msg){
        // TO DO: given msg, for each wheel, retrieve the wheel velocity in rad/s
        // (probably we will save them in an array)


        float robot_vel[3] = computeRobotVelocity(wheels_vel);

        auto vel_msg = geometry_msgs::TwistStamped();
        vel_msg.header.frame_id = "";
        vel_msg.header.stamp = ros::Time::now();

        vel_msg.twist.linear = toVector3(robot_vel[1], robot_vel[2], 0);
        vel_msg.twist.angular = toVector3(0, 0, robot_vel[0]);

        pub.publish(vel_msg);
    }

    // Compute robot linear and angular velocity from wheel angular velocities
    float[] computeRobotVelocity(float wheels_vel[4]) {
        float robot_vel[3] = {0, 0, 0};

        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 4; j++) {
                robot_vel[i] += inv_H0[i][j] * wheels_vel[j];
            }
        }

        return robot_vel;
    }

    // Compute the inverse of H(0) given wheels l and w (global parameters)
    float[][] inverseH0(float r, float l, float w) {
        float inv_H0[3][4] = {{-0.25/(l+w), 0.25/(l+w), 0.25/(l+w), -0.25/(l+w)},
                                {0.25, 0.25, 0.25, 0.25},
                                {-0.25, -0.25, -0.25, -0.25}};
        
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 4; j++) {
                inv_H0[i][j] = r * inv_H0[i][j];
            }
        }
        return inv_H0;
    }

    // Convert 3 floats into Vector3 (used in geometry_msgs::TwistStamped)
    geometry_msgs::Vector3 toVector3(float x, float y, float z) {
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

    float r;
    float l;
    float w;
    float inv_H0[3][4];
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity");
    Velocity velocity;
    ros::spin();
    return 0;
}
