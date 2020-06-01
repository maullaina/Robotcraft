#include <iostream>

#include <cstdlib>
#include <stdbool.h>
#include <tf/transform_datatypes.h>

#include "std_msgs/Float32.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884L
#endif na

float r;
float x, y, theta;

float desired_ang_vel, correction_factor;

class SquareTest
{
private:
    ros::NodeHandle n;
    ros::Publisher square_vel_pub;
    ros::Subscriber ir_front_sensor_sub;
    ros::Subscriber ir_left_sensor_sub;
    ros::Subscriber ir_right_sensor_sub;
    ros::Subscriber odom_sub;



    geometry_msgs::Twist calculateCommand()
    {
        //Variable to store original position
        static float x_orig , y_orig, theta_orig;

        static bool init_flag = false;
        static bool done_driving_forward_flag = false;
        static bool start_turning_flag = false;


        if(!init_flag){
          x_orig = 0;
          y_orig = 0;
          theta_orig = 0;
          init_flag = true;
        }

        auto square_vel_msg = geometry_msgs::Twist();

        //std::cout<<"Theta current: "<<theta<<"\n";


        float distance = sqrt((x_orig - x)*(x_orig - x) + (y_orig - y)*(y_orig - y));
        if (distance<0.5){
          square_vel_msg.linear.x = 0.05; // m/0.1s
          square_vel_msg.angular.z = 0;
          theta_orig = theta;
        }else{
          done_driving_forward_flag = true;
          start_turning_flag = true;
        }


        if(start_turning_flag){
            if (theta_orig > M_PI/2  &&  theta<M_PI/2){
              theta = M_PI +(M_PI + theta);
            }
          if(fabs(theta - theta_orig) <= correction_factor*M_PI/2){
            square_vel_msg.angular.z = desired_ang_vel;
          }else{
            // When the turn is finished update the reference position
            x_orig = x;
            y_orig = y;
            distance = 0;
            start_turning_flag = false;
            done_driving_forward_flag = false;
          }
        }


        return square_vel_msg;
    }


    void frontSensorCallback(const sensor_msgs::Range::ConstPtr &msg) {
      if(msg->range < 0.15){
        ROS_DEBUG("Collision risk! The robot is %f meters of an obsctacle, on the front side");
      }
    }

    void rightSensorCallback(const sensor_msgs::Range::ConstPtr &msg) {
      if(msg->range < 0.15){
        ROS_DEBUG("Collision risk! The robot is %f meters of an obsctacle, on the right side");
      }
    }

    void leftSensorCallback(const sensor_msgs::Range::ConstPtr &msg) {
      if(msg->range < 0.15){
        ROS_DEBUG("Collision risk! The robot is %f meters of an obsctacle, on the left side");
      }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
      tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  		tf::Matrix3x3 m(q);
  		double roll, pitch, yaw;
  		m.getRPY(roll, pitch, yaw);
      x = msg->pose.pose.position.x;
      y = msg->pose.pose.position.y;
      theta = yaw;


    }


public:
    SquareTest(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages to square_vel_pub
        this->square_vel_pub = this->n.advertise<geometry_msgs::Twist>("square_vel_pub", 10);

        // Create a subscriber for position
        //CHANGE BACK TO ODOM_PUB FOR ARDUINO
        this->odom_sub = n.subscribe("odom", 10, &SquareTest::odomCallback, this);
        this->ir_front_sensor_sub = n.subscribe("ir_front_sensor", 10, &SquareTest::frontSensorCallback, this);
        this->ir_right_sensor_sub = n.subscribe("ir_right_sensor", 10, &SquareTest::rightSensorCallback, this);
        this->ir_left_sensor_sub = n.subscribe("ir_left_sensor", 10, &SquareTest::leftSensorCallback, this);

        this->n.getParam("/desired_ang_vel", desired_ang_vel);
        this->n.getParam("/correction_factor", correction_factor);
        //this->n.getParam("/p", p);
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // Calculate the command to drive in squares
            auto square_vel_msg = calculateCommand();

            // Publish the new command
            this->square_vel_pub.publish(square_vel_msg);

            // Go through buffer
            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "square_test");


    // Create our controller object and run it
    auto controller = SquareTest();
    controller.run();
}
