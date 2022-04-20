#include "ros/ros.h"
#include <array>

class Odometry{

public:
    Odometry() { //variables in which I have to save initialpose
        this->prev_t = 0;
        this->firstTime = false;
        sub_v = n.subscribe("/cmd_vel", 1000, &Odometry::odometryCallback, this);
        sub_p = n.subscribe("/robot/pose", 1000, &Odometry::odometryCallback, this);
        pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg_p){
        if(!firstTime){
            currx = msg_p->pose.position.x;
            curry = msg_p->pose.position.y;

            tf::Quaternion q{
                msg_p->pose.orientation.x;
                msg_p->pose.orientation.y;
                msg_p->pose.orientation.z;
                msg_p->pose.orientation.w;
            }
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            currTheta = yaw;
            firstTime = true;
        }
    }

    void odometryCallback(const geometry_msgs::TwistStamped::ConstPtr& msg_v) {

        //get dei parametri della pose attuale,
        //che ho settato con l'odometryCallback di prima.
        auto odom_msg = nav_msgs::Odometry;
        odom_msg.header.frame_id = "";
        odom_msg.header.stamp = ros::Time::now();

        //If the current time is not the first (so if it is not 0), I compute the
        //odometry. If it is the first then I do nothing and I store the value in
        //a prev_t variable, which contains the previous time and is updated at
        //the end of every cycle.

        odom_msg.pose.pose = computeEulerOdometry(msg_v->twist.linear, msg_v->twist.angular,
            msg_v->header.stamp,
            this->prev_t,
            currx, curry, currTheta);



        odom_msg.twist.twist.linear = msg_v->twist.linear;
        odom_msg.twist.twist.angular = msg_v->twist.angular;

        pub.publish(odom_msg);

        this->prev_t = msg_p->header.stamp;

    }

        //Basic Euler odometry. Needs to be updated once I learn how to use quaternions.
        geometry_msgs::Pose computeEulerOdometry(geometry_msgs::Vector3 vel_lin,
            geometry_msgs::Vector3 vel_ang,
            time timestamp,
            time prev_t,
            float64 x, float64 y, float64 theta){

                auto odom_pos = geometry_msgs::Pose();

                t = timestamp - prev_t; //sampling period t

                odom_pos.point.x =  x + vel_lin.x*t;
                odom_pos.point.y =  y + vel_lin.y*t;
                odom_pos.point.z = 0; //it's alwyas 0, it moves on a plane

                currTheta = currTheta * vel_ang.z*t

                tf2::Quaternion q;
                q.setRPY(0,0, currTheta);
                odom_pos.orientation.x = q.x();
                odom_pos.orientation.y = q.y();
                odom_pos.orientation.z = q.z();
                odom_pos.orientation.w = q.w();

                currx = odom_pos.point.x;
                curry = odom_pos.point.y;

                return odom_pos;
            }


        private:
            ros::NodeHandle n;
            nav_msgs::Odometry robot_odometry;
            ros::Subscriber sub;
            ros::Publisher pub;
            bool firstTime;
            time prev_t;
            float64 currx;
            float64 curry;
            float64 currTheta;
        }

        int main(int argc, char**argv){
            ros::init(argc, argv, "odometry");
            Odometry odometry;
            while(ros::ok){
                //loop stuff
            }
            return 0;
        }
