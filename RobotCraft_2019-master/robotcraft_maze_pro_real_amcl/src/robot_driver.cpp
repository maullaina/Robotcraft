#include <iostream>
#include <cstdlib>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <rosserial_arduino/Test.h>
#include <string>
#include <list>
#include <tuple>

#include <vector>
#include <algorithm>
#include <utility>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"

#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"


#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884L
#endif na


class RobotDriver
{
private:
    //Initialising node NodeHandle
    ros::NodeHandle n;

    //Keep track of current ctime
    ros::Time current_time;
    ros::Time last_time = ros::Time::now();


    //Declare transform broadcaster to send messages via tf and ROS
    tf::TransformBroadcaster odom_broadcaster;

    double v;
    double w;
    float initial_x, initial_y, initial_theta;

    int endX, endY;
    int startX, startY;


    //Publisher Topics
    ros::Publisher set_pose_pub;
    ros::Publisher odom_pub;

    //Subscriber Topics
    ros::Subscriber pose_sub;


    //Callback functions for subscribers
    void poseCallback(const geometry_msgs::Pose2D& pose_msg){
        current_time = ros::Time::now();

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_msg.theta);

        //first, we'll publish the transform over tf - broadcasts to rviz
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = pose_msg.x;
        odom_trans.transform.translation.y = pose_msg.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = pose_msg.x;
        odom.pose.pose.position.y = pose_msg.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = v;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = w;

        //publish the message
        odom_pub.publish(odom);
        last_time = current_time;

    }

    geometry_msgs::Pose2D setPose(){
        auto pose_MSG = geometry_msgs::Pose2D();
        // Set position to zero
        pose_MSG.x = initial_x;
        pose_MSG.y = initial_y;
        pose_MSG.theta = initial_theta;
        return pose_MSG;
    }


public:
  //constructor
    RobotDriver(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        this->n.getParam("/initial_x", initial_x);
        this->n.getParam("/initial_y", initial_y);
        this->n.getParam("/initial_theta", initial_theta);

        // Create a publisher object, able to push messages
        this->set_pose_pub = this->n.advertise<geometry_msgs::Pose2D>("set_pose", 2);
		this->odom_pub = this->n.advertise<nav_msgs::Odometry>("odom", 10);
        // Create a subscriber for laser scans
        this->pose_sub = n.subscribe("pose", 10, &RobotDriver::poseCallback, this);


    }



    void run(){
        int count = 0;

        // Send messages in a loop
        ros::Rate loop_rate(2);

        while (ros::ok())
        {

            // Calculate the command to apply
            auto pose_MSG = setPose();

            // Publish the new command
            this->set_pose_pub.publish(pose_MSG);


            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){

    // Initialize ROS
    ros::init(argc, argv, "robot_driver");

    // Create our controller object and run it
    auto controller = RobotDriver();
    controller.run();
}
