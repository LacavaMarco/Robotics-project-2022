#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

class TfBroad {

public:
    TfBroad() {
        sub_reset = n.subscribe("reset_pose", 1000, &TfBroad::resetCallback, this);
        sub_o = n.subscribe("odom", 1000, &TfBroad::odomCallback, this);
    }

    // Initialize pose with the values of the first message of /robot/pose topic.
    // Also used to reset the pose when requested by resetpose service: first updates
    // the position of frame "odom" wrt "map", then set the position of "base_link"
    // in the origin of "odom"
    void resetCallback(const geometry_msgs::Vector3::ConstPtr& msg_reset) {
        initial_x = msg_reset->x;
        initial_y = msg_reset->y;
        initial_theta = msg_reset->z;
        // set header
        transformStampedPose.header.stamp = ros::Time::now();
        transformStampedPose.header.frame_id = "map";
        transformStampedPose.child_frame_id = "odom";
        // set x,y
        transformStampedPose.transform.translation.x = msg_reset->x;
        transformStampedPose.transform.translation.y = msg_reset->y;
        transformStampedPose.transform.translation.z = 0.0;
        // set theta
        tf2::Quaternion q;
        q.setRPY(0, 0, msg_reset->z);
        transformStampedPose.transform.rotation.x = q.x();
        transformStampedPose.transform.rotation.y = q.y();
        transformStampedPose.transform.rotation.z = q.z();
        transformStampedPose.transform.rotation.w = q.w();
        // send transform
        pose_br.sendTransform(transformStampedPose);

        // ROS_INFO("Pose reset to: (%f, %f, %f)", x, y, theta);

        // set header
        transformStampedOdom.header.stamp = ros::Time::now();
        transformStampedOdom.header.frame_id = "odom";
        transformStampedOdom.child_frame_id = "base_link";
        // set x,y
        transformStampedOdom.transform.translation.x = 0.0;
        transformStampedOdom.transform.translation.y = 0.0;
        transformStampedOdom.transform.translation.z = 0.0;
        // set theta
        tf2::Quaternion p;
        p.setRPY(0, 0, 0);
        transformStampedPose.transform.rotation.x = p.x();
        transformStampedPose.transform.rotation.y = p.y();
        transformStampedPose.transform.rotation.z = p.z();
        transformStampedPose.transform.rotation.w = p.w();
        // send transform
        odom_br.sendTransform(transformStampedOdom);
    }

    // Update position of frame "base_link" wrt "odom" (somehow if I don't send
    // the tf map->odom it can't be correctly visualized in rviz)
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // set header
        transformStampedPose.header.stamp = ros::Time::now();
        transformStampedPose.header.frame_id = "map";
        transformStampedPose.child_frame_id = "odom";
        // set x,y
        transformStampedPose.transform.translation.x = initial_x;
        transformStampedPose.transform.translation.y = initial_y;
        transformStampedPose.transform.translation.z = 0.0;
        // set theta
        tf2::Quaternion q;
        q.setRPY(0, 0, initial_theta);
        transformStampedPose.transform.rotation.x = q.x();
        transformStampedPose.transform.rotation.y = q.y();
        transformStampedPose.transform.rotation.z = q.z();
        transformStampedPose.transform.rotation.w = q.w();
        // send transform
        pose_br.sendTransform(transformStampedPose);

        // set header
        transformStampedOdom.header.stamp = ros::Time::now();
        transformStampedOdom.header.frame_id = "odom";
        transformStampedOdom.child_frame_id = "base_link";
        // set x,y
        transformStampedOdom.transform.translation.x = msg->pose.pose.position.x;
        transformStampedOdom.transform.translation.y = msg->pose.pose.position.y;
        transformStampedOdom.transform.translation.z = 0.0;
        // set theta
        transformStampedOdom.transform.rotation.x = msg->pose.pose.orientation.x;
        transformStampedOdom.transform.rotation.y = msg->pose.pose.orientation.y;
        transformStampedOdom.transform.rotation.z = msg->pose.pose.orientation.z;
        transformStampedOdom.transform.rotation.w = msg->pose.pose.orientation.w;
        // send transform
        odom_br.sendTransform(transformStampedOdom);
    }

private:
    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster odom_br;
    tf2_ros::TransformBroadcaster pose_br;
    geometry_msgs::TransformStamped transformStampedOdom;
    geometry_msgs::TransformStamped transformStampedPose;
    ros::Subscriber sub_reset;
    ros::Subscriber sub_o;

    // Store the current position of frame "odom" wrt "map"
    double initial_x, initial_y, initial_theta;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_broadcast");
    TfBroad my_tf_broadcaster;
    ros::spin();
    return 0;
}
