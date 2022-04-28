#include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

class TfBag {

public:
    TfBag() {
        sub = n.subscribe("/robot/pose", 1000, &TfBag::callback, this);
    }

    void callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        // set header
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "bag";
        // set x,y
        transformStamped.transform.translation.x = msg->pose.position.x;
        transformStamped.transform.translation.y = msg->pose.position.y;
        transformStamped.transform.translation.z = 0.0;
        // set theta
        transformStamped.transform.rotation.x = msg->pose.orientation.x;
        transformStamped.transform.rotation.y = msg->pose.orientation.y;
        transformStamped.transform.rotation.z = msg->pose.orientation.z;
        transformStamped.transform.rotation.w = msg->pose.orientation.w;
        // send transform
        br.sendTransform(transformStamped);
    }

private:
    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    ros::Subscriber sub;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_bag");
    TfBag tf_bag;
    ros::spin();
    return 0;
}
