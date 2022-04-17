#include "ros/ros.h"

class Odometry{

public:
  Odometry() { //variables in which I have to save initialpose
    this->prev_t = 0;
    sub_v = n.subscribe("/cmd_vel", 1000, &Odometry::odometryCallback, this);
    sub_p = n.subscribe("/robot/pose", 1000, &Odometry::odometryCallback, this);
    pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
  }

  void odometryCallback(const geometry_msgs::TwistStamped::ConstPtr& msg_v,
                        const geometry_msgs::PoseStamped::ConstPtr& msg_p) {
    //get dei parametri della pose attuale,
    //che ho settato con l'odometryCallback di prima.
    int lin_vel[3]= msg_v->twist.linear;
    int ang_vel[3]= msg_v->twist.angular;
    auto odom_msg = nav_msgs::Odometry;
    odom_msg.header.frame_id = "";
    odom_msg.header.stamp = ros::Time::now();

    //If the current time is not the first (so if it is not 0), I compute the
    //odometry. If it is the first then I do nothing and I store the value in
    //a prev_t variable, which contains the previous time and is updated at
    //the end of every cycle.
    if(msg_p->header.stamp != 0){
      odom_msg.pose.pose = computeEulerOdometry(msg_v->twist.linear, msg_v->twist.angular,
                                        msg_p->header.stamp,
                                        this->prev_t,
                                        msg_p->pose.position, msg_p->pose.orientation);
      pub.publish(odom_msg);
    }

    this->prev_t = msg_p->header.stamp;

  }

  //Basic Euler odometry. Needs to be updated once I learn how to use quaternions.
  geometry_msgs::Pose computeEulerOdometry(geometry_msgs::Vector3 x,
                                      geometry_msgs::Vector3 y,
                                      time timestamp,
                                      time prev_t,
                                      geometry_msgs::Point initPose
                                      geometry_msgs::Quaternion initOrientation){

      auto odom_pos = geometry_msgs::Point();

      t = timestamp - prev_t; //sampling period t

      odom_pos.point.x =  initPose.x + x*t*3.14; //instead of 3.14 there should be cos(Theta)
      odom_pos.point.y =  initPose.y + y*t*3.14; //instead of 3.14 there should be sin(Theta)
      odom_pos.point.z = 0; //it's alwyas 0, it moves on a plane

      //needs to be theta as well here, but it's in quaternions (still need to
      // figure it out)
  }


private:
  ros::NodeHandle n;
  nav_msgs::Odometry robot_odometry;
  ros::Subscriber sub;
  ros::Publisher pub;

  time prev_t;


}

int main(int argc, char**argv){
  ros::init(argc, argv, "odometry");
  Odometry odometry;
  while(ros::ok){
    //loop stuff
  }
  return 0;
}
