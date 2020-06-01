#include <iostream>
#include <cstdlib>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <rosserial_arduino/Test.h>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884L
#endif na



class ReactiveController
{
private:
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub;

    ros::Subscriber ir_front_sensor;
    ros::Subscriber ir_left_sensor;
    ros::Subscriber ir_right_sensor;

    double front_obstacle_distance;
    double right_obstacle_distance;
    double left_obstacle_distance;
    bool robot_stopped;

    // Desired distance to the right wall
    float desired_right_wall_distance;
    float front_obstacle_distance_threshold;

    // Wall distance controller parameters
    int k_p;
    int k_i;


    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();

        float dt = 0.1;


        static float error;
        static float integral_error;

        float proportional;
        float integral;
        float angular_input;

        // Switch case variables
        static bool init_flag = false;
        static int swtich_case = 0;

        static int count = 0;

        
        std::cout << "Front_obstacle_distance: " << front_obstacle_distance  << "\n";

        if(!init_flag){
        	// INIT
        	swtich_case = 0;
        } else if (front_obstacle_distance > 0.6){
        	// PID
        	swtich_case = 1;
        } else{
        	// MANUEVER
        	swtich_case = 2;
        }
        

        switch(swtich_case) {

            case 0 :        // Initialisation
            				if(count < 300){
                                msg.linear.x = 0.0;
                                count++;
                            } else if(front_obstacle_distance > 0.4){
                                msg.linear.x = 0.6;
                            } else {
                                init_flag = true;
                            }
                            break;       // and exits the switch
          
            case 1 :       	// PI controller

        					// Calculate proportional part
        					error = desired_right_wall_distance - right_obstacle_distance;
        					proportional =  error*k_p;

        					// Calculate integral part
        					integral_error += error*dt;
        					integral = integral_error*k_i;

        					// Calculate final angular velocity input
        					angular_input = integral + proportional;

        					// Write velocities for the next time step
        					msg.angular.z = angular_input;

        					if(front_obstacle_distance < front_obstacle_distance_threshold){
        						msg.linear.x = 0.05;
        					} else{
        						msg.linear.x = 0.07;
        					}

        					std::cout << "Case PID \n";
                            break;

            case 2 :		// Avoidance manuever
                    		msg.linear.x = 0.00001;
                    		msg.angular.z = M_PI*5;
                    		
                    		error = 0;
                    		integral_error = 0;

                    		std::cout << "Case MANUEVER \n";
                            break;
        }
        
        // Wirte message to the robot
        return msg;

    }
    

    void ir_right_Callback(const sensor_msgs::Range::ConstPtr& ir_msg)
    {

        right_obstacle_distance = ir_msg->range;
        
    }
    void ir_left_Callback(const sensor_msgs::Range::ConstPtr& ir_msg)
    {

        left_obstacle_distance = ir_msg->range;
        
    }
    void ir_front_Callback(const sensor_msgs::Range::ConstPtr& ir_msg)
    {

        front_obstacle_distance = ir_msg->range;
        
    }


public:
    ReactiveController(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("reactive_vel", 10);

        // Create a subscriber for laser scans 
        //this->laser_sub = n.subscribe("base_scan", 10, &ReactiveController::laserCallback, this);

        // name_of_the_subscriber = n.subscribe("topic_name")
        this->ir_right_sensor = n.subscribe("ir_right_sensor", 10, &ReactiveController::ir_right_Callback, this);
        this->ir_left_sensor = n.subscribe("ir_left_sensor", 10, &ReactiveController::ir_left_Callback, this);
        this->ir_front_sensor = n.subscribe("ir_front_sensor", 10, &ReactiveController::ir_front_Callback, this);

        this->n.getParam("/desired_right_wall_distance", desired_right_wall_distance);
        this->n.getParam("/front_obstacle_distance_threshold", front_obstacle_distance_threshold);
        this->n.getParam("/k_p", k_p);
        this->n.getParam("/k_i", k_i);
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // Calculate the command to apply
            auto msg = calculateCommand();

            // Publish the new command
            this->cmd_vel_pub.publish(msg);

            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "reactive_controller");


    // Create our controller object and run it
    auto controller = ReactiveController();
    controller.run();
}