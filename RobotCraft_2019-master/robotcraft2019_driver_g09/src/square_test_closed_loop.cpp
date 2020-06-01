#include <iostream>

#include <cstdlib>
#include <stdbool.h>
#include <tf/transform_datatypes.h>

#include "std_msgs/Float32.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"


#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884L
#endif na

float r;
float x, y, theta;

class SquareTest
{
private:
    ros::NodeHandle n;
    ros::Publisher square_vel_pub;
    ros::Subscriber odom_sub;

  float v;
	float w;
	float K_omega; // Linear error
	float K_psi;    // Angular error
	float p;		 // Recovery speed during line following
	float theta_ref;

	float goal_pos_arr[4][2] = {
		{0.0,0.0},
		{1.0,0.0},
		{1.0,1.0},
		{0.0,1.0}
	};

	enum CONTROLLER_STATE
	{   ROTATE = 1,
	    FOLLOW_LINE = 2,
	};



    geometry_msgs::Twist calculateCommand()
    {
    	auto square_vel_msg = geometry_msgs::Twist();

        //Variable to store original position
        static float x_start , y_start, x_goal, y_goal;
        static bool init_flag = false;
        static bool done_flag = true;
        static int i = 0;
        static CONTROLLER_STATE ControllerState;




        if(!init_flag){
          x_start = 0;
          y_start = 0;
          init_flag = true;
          ControllerState = ROTATE;
        }

        if(done_flag){
        	if(i == 4){
        		i = 0;
        	}
        	x_goal = goal_pos_arr[i][0];
        	y_goal = goal_pos_arr[i][1];
        	i++;
        	done_flag = false;
        }

        //std::cout<<"Theta current: "<<theta<<"\n";


        theta_ref = atan2((y_goal-y),(x_goal-x));
        std::cout << "x_goal " << x_goal << " y_goal"  << y_goal <<"\n";
		std::cout << "x_curr " << x << " y_curr"  << y <<"\n";
		std::cout << "Theta_ref " << theta_ref << " Theta curr"  << theta <<"\n";


		switch (ControllerState) {

		  case 1 :
		    std::cout << "ROTATE" << "\n";
		    std::cout << "Diff" << (theta - theta_ref) <<"\n";
		  	if (theta - theta_ref > 0.05 || theta - theta_ref < -0.05 ){
		  		v = K_omega * (cos(theta)*(x_start-x) + sin(theta)*(y_start-y));
				w = K_psi*(theta_ref-theta);
		  	} else{
		  		w = 0;
		  		v = 0;
		  		ControllerState = FOLLOW_LINE;
		  		x_start = x;
		  		y_start = y;
		  	}
		    break;

		  case 2 :
		    std::cout << "FOLLOW LINE" << "\n";
		    if(cos(theta)*(x_goal-x) + sin(theta)*(y_goal-y) > 0.05 || cos(theta)*(x_goal-x) + sin(theta)*(y_goal-y) < -0.05 ){
		  		w = K_psi*(sin(theta_ref)*(x + p*cos(theta)-x_start) - cos(theta_ref)*(y + p*sin(theta)- y_start));
				v = K_omega * (cos(theta)*(x_goal-x) + sin(theta)*(y_goal-y));
		  	} else{
		  		w = 0;
		  		v = 0;
		  		done_flag = true;
		  		x_start = x;
		  		y_start = y;
		  		ControllerState = ROTATE;
		  	}
		    break;
		}

		square_vel_msg.linear.x = v;
		square_vel_msg.angular.z = w;
        return square_vel_msg;
    }


    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
		tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		x = msg->pose.pose.position.x;
		y = msg->pose.pose.position.y;
		theta = yaw;
		std::cout << "YAW" << yaw << "\n";
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

        this->n.getParam("/K_psi", K_psi);
        this->n.getParam("/K_omega", K_omega);
        this->n.getParam("/p", p);



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
