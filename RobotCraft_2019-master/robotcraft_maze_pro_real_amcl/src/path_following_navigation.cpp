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
#include "geometry_msgs/PoseWithCovarianceStamped.h"
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



class PathFollowingController
{
private:
    ros::NodeHandle n;

    ros::Publisher path_follow_vel_pub;
    ros::Subscriber map_sub;
    ros::Subscriber odom_sub;

    int Width;
    int Height;
    float cell_size;
    float max_ang = 0.5;
    float max_lin = 0.055;

    int endX, endY;
    int startX, startY;

    float x_real=-999.0, y_real=-999.0, theta_real=-999.0;


    bool robot_stopped;

    // Desired distance to the right wall
    float desired_linear_velocity;
    float desired_side_wall_distance;
    float front_obstacle_distance_threshold;
    float desired_wall;

    // Wall distance controller parameters
    float v;
    float w;
    float K_omega; // Linear error
    float K_psi;    // Angular error
    float p;         // Recovery speed during line following
    float theta_ref;

    //float x_offset, y_offset;


    enum CONTROLLER_STATE
    {   ROTATE = 1,
        FOLLOW_LINE = 2,
    };

     std::list<std::pair<int,int>> final_path;


    bool init_flag = false;
    bool path_set = false;
    bool path_done = false;
    bool pos_set = false;

    std::pair<int,int> my_p;


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
          //my_p = final_path.front();
          //final_path.pop_front();
          std::cout << "FIRST GOAL" << (final_path.front()).first << "\n";
        }

        if(done_flag){
            if(final_path.empty()){
                //std::cout << "List is empty\n" ;
            }else{
                my_p = final_path.front();
                final_path.pop_front();
                x_goal = Width*cell_size-(Width-(my_p.second))*cell_size+cell_size/2;
                y_goal = Height*cell_size-(my_p.first)*cell_size-cell_size/2;
            }
            done_flag = false;
        }

        //std::cout<<"Theta current: "<<theta<<"\n";


        theta_ref = atan2((y_goal-y_real),(x_goal-x_real));
        std::cout << "x_goal " << x_goal << " y_goal"  << y_goal <<"\n";
        std::cout << "x_curr " << x_real << " y_curr"  << y_real <<"\n";
        std::cout << "Theta_ref " << theta_ref << " Theta curr"  << theta_real <<"\n";


        switch (ControllerState) {

          case 1 :
            std::cout << "ROTATE" << "\n";
            //std::cout << "Diff" << (theta - theta_ref) <<"\n";
            if (theta_real - theta_ref > 0.15 || theta_real - theta_ref < -0.15 ){
                v = 0;//K_omega * (cos(theta_real)*(x_start-x_real) + sin(theta_real)*(y_start-y_real));
                w = K_psi*(theta_ref-theta_real);
            } else{
                w = 0;
                v = 0;
                ControllerState = FOLLOW_LINE;
                x_start = x_real;
                y_start = y_real;
            }

            if(w > 1.2){
                w = 1.2;
            } else if(w < -1.2){
                  w = -1.2;
            }
            break;

          case 2 :
            std::cout << "FOLLOW LINE" << "\n";
            if(cos(theta_real)*(x_goal-x_real) + sin(theta_real)*(y_goal-y_real) > 0.02 || cos(theta_real)*(x_goal-x_real) + sin(theta_real)*(y_goal-y_real) < -0.02 ){
                w = K_psi*(sin(theta_ref)*(x_real + p*cos(theta_real)-x_start) - cos(theta_ref)*(y_real + p*sin(theta_real)- y_start));
                v = K_omega * (cos(theta_real)*(x_goal-x_real) + sin(theta_real)*(y_goal-y_real));
            } else{
                w = 0;
                v = 0;
                done_flag = true;
                x_start = x_real;
                y_start = y_real;
                ControllerState = ROTATE;
            }

            if(w > max_ang){
                w = max_ang;
            } else if(w < -max_ang){
                  w = -max_ang;
            }

            break;
        }


        if(v > max_lin){
            v = max_lin;
        } else if(v < -max_lin){
              v = -max_lin;
        }

        square_vel_msg.linear.x = v;
        square_vel_msg.angular.z = w;
        return square_vel_msg;
    }



    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        std_msgs::Header header = msg->header;
        nav_msgs::MapMetaData info = msg->info;
        ROS_INFO("Got map %d %d", info.width, info.height);

        cell_size = info.resolution;
        Width = info.width, Height = info.height;
        int cols = info.width, rows = info.height;
        auto p = [&](int x, int y) { return x * cols + y;  };

        //Binary map
        int** M = new int*[rows];
        for (int i = 0; i < rows; ++i){
            M[i] = new int[cols];
        }

        //Potential filed
        int** C = new int*[rows];
        for (int i = 0; i < rows; ++i){
            C[i] = new int[cols];
        }

        //Shortest path
        int** P = new int*[rows];
        for (int i = 0; i < rows; ++i){
            P[i] = new int[cols];
        }

        std::cout << "\n\n BINARY MAP \n\n";
        for (int i = (rows-1); i >= 0; --i) {   // for each row
            for (int j = 0; j < cols; ++j) { // for each column
                if(msg->data[i*cols + j] > 0){
                    M[i][j] = 1;
                } else{
                    M[i][j] = 0;
                }
                std::cout << M[i][j] << " ";
            }
            std::cout << "\n";
        }

        for(int x = (rows-1); x >= 0; --x){
            for(int y = 0; y < cols; y++){
                if(x == 0 || y == 0 || x == (rows -1) || y == (cols -1) || M[x][y] == 1){
                    C[rows-1-x][y] = -1;
                }else{
                    C[rows-1-x][y] = 0;
                }
            }
        }




        std::list<std::tuple<int,int,int>> nodes;

        nodes.push_back({endX,endY,1});

        while(!nodes.empty()){
            std::list<std::tuple<int,int,int>> new_nodes;

            for(auto &n : nodes){
                int x = std::get<0>(n);
                int y = std::get<1>(n);
                int d = std::get<2>(n);

                C[x][y] = d;

                // Check south
                if((x+1) < rows && C[x+1][y] == 0){
                    new_nodes.push_back({x+1,y,d+1});
                }
                // Check north
                if((x-1) >= 0 && C[x-1][y] == 0){
                    new_nodes.push_back({x-1,y,d+1});
                }
                // Check east
                if((y+1) < cols && C[x][y+1] == 0){
                    new_nodes.push_back({x,y+1,d+1});
                }
                // Check west
                if((y-1) >= 0 && C[x][y-1] == 0){
                    new_nodes.push_back({x,y-1,d+1});
                }
                /*
                // Check south east
                if((x+1) < rows && (y+1) < cols && C[x+1][y+1] == 0){
                    new_nodes.push_back({x+1,y+1,d+1});
                }
                // Check north east
                if((x-1) >= 0 && (y+1) < cols && C[x-1][y+1] == 0){
                    new_nodes.push_back({x-1,y+1,d+1});
                }
                // Check south west
                if((x+1) < rows && (y-1) >= 0 && C[x+1][y-1] == 0){
                    new_nodes.push_back({x+1,y-1,d+1});
                }
                // Check north west
                if((x-1) >= 0 && (y-1) >= 0  && C[x-1][y-1] == 0){
                    new_nodes.push_back({x-1,y-1,d+1});
                }

                */
            }
            // Sort the nodes - This will stack up nodes that are similar: A, B, B, B, B, C, D, D, E, F, F
            new_nodes.sort([&](const std::tuple<int, int, int> &n1, const std::tuple<int, int, int> &n2)
            {
                // In this instance I dont care how the values are sorted, so long as nodes that
                // represent the same location are adjacent in the list. I can use the p() lambda
                // to generate a unique 1D value for a 2D coordinate, so I'll sort by that.
                return p(std::get<0>(n1), std::get<1>(n1)) < p(std::get<0>(n2), std::get<1>(n2));
            });

            // Use "unique" function to remove adjacent duplicates       : A, B, -, -, -, C, D, -, E, F -
            // and also erase them                                       : A, B, C, D, E, F
            new_nodes.unique([&](const std::tuple<int, int, int> &n1, const std::tuple<int, int, int> &n2)
            {
                return  p(std::get<0>(n1), std::get<1>(n1)) == p(std::get<0>(n2), std::get<1>(n2));
            });

            // We've now processed all the discoverd nodes, so clear the list, and add the newly
            // discovered nodes for processing on the next iteration
            nodes.clear();
            nodes.insert(nodes.begin(), new_nodes.begin(), new_nodes.end());
        }


        std::cout << "\n\n VISUALIZABLE BINARY \n\n";


        for(int x = 0; x < rows; x++){
            for(int y = 0; y < cols; y++){
                //std::cout << C[x][y] << " ";
                if(C[x][y] == -1){
                    std::cout << "X ";
                }
                if(C[x][y] >= 0){
                    std::cout << "- ";
                }
            }
            std::cout << "\n";
        }


        std::list<std::pair<int,int>> path;
        path.push_back({startX,startY});
        int locX = startX;
        int locY = startY;
        bool no_path = false;

        while(!(locX == endX && locY == endY) && !no_path){
            std::list<std::tuple<int,int,int>> listNeighbours;

            // Check south
            if((locX+1) < rows && C[locX+1][locY] > 0){
                listNeighbours.push_back({locX+1,locY,C[locX+1][locY]});
            }
            // Check north
            if((locX-1) >= 0 && C[locX-1][locY] > 0){
                listNeighbours.push_back({locX-1,locY,C[locX-1][locY]});
            }
            // Check east
            if((locY+1) < cols && C[locX][locY+1] > 0){
                listNeighbours.push_back({locX,locY+1,C[locX][locY+1]});
            }
            // Check west
            if((locY-1) >= 0 && C[locX][locY-1] > 0){
                listNeighbours.push_back({locX,locY-1,C[locX][locY-1]});
            }
            /*
            // Check south east
            if((locX+1) < rows && (locY+1) < cols && C[locX+1][locY+1] > 0){
                listNeighbours.push_back({locX+1,locY+1,C[locX+1][locY+1]});
            }
            // Check north east
            if((locX-1) >= 0 && (locY+1) < cols && C[locX-1][locY+1] > 0){
                listNeighbours.push_back({locX-1,locY+1,C[locX-1][locY+1]});
            }
            // Check south west
            if((locX+1) < rows && (locY-1) >= 0 && C[locX+1][locY-1] > 0){
                listNeighbours.push_back({locX+1,locY-1,C[locX+1][locY-1]});
            }
            // Check north west
            if((locX-1) >= 0 && (locY-1) >= 0  && C[locX-1][locY-1] > 0){
                listNeighbours.push_back({locX-1,locY-1,C[locX-1][locY-1]});
            }
			*/

            listNeighbours.sort([&](const std::tuple<int, int, int> &n1, const std::tuple<int, int, int> &n2)
            {
                return std::get<2>(n1) < std::get<2>(n2); // Compare distances
            });

            if (listNeighbours.empty()) // Neighbour is invalid or no possible path
                no_path = true;
            else
            {
                locX = std::get<0>(listNeighbours.front());
                locY = std::get<1>(listNeighbours.front());
                path.push_back({ locX, locY });
            }

        }
        P = C;
        final_path = path;
        int p_x, p_y;
        for (auto &a : path){
            p_x = a.first;
            p_y = a.second;
            std::cout << "X" << p_x << " Y " << p_y << "\n";
            P[p_x][p_y] = 0;
        }



        std::cout << "\n\n SHORTEST PATH \n\n";

        for(int x = 0; x < rows; x++){
            for(int y = 0; y < cols; y++){
                if(P[x][y] == -1){
                    std::cout << "X ";
                }
                if(P[x][y] == 0){
                    std::cout << "O ";
                }
                if(P[x][y] > 0){
                    std::cout << "- ";
                }
            }
            std::cout << "\n";
        }

        final_path = path;
        path_set = true;

        /*

        nav_msgs::OccupancyGrid* newGrid = map.Grid();
        newGrid->header = header;
        newGrid->info = info;
        map_pub.publish(*newGrid);
        */

        for (int i = 0; i < rows; ++i){
            delete [] M[i];
        }
        delete [] M;

        for (int i = 0; i < rows; ++i){
            delete [] C[i];
        }
        delete [] C;


    }

    void odomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        x_real = (msg->pose.pose.position.x); //0.2
        y_real = (msg->pose.pose.position.y); //5.175;
        theta_real = yaw;
        //std::cout << "YAW" << yaw << "\n";
        if(x_real < 0.001){
        	pos_set = false;
        }else{
        	pos_set = true;
        }
    }

public:
    PathFollowingController(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages
        this->path_follow_vel_pub = this->n.advertise<geometry_msgs::Twist>("reactive_vel", 2);

        // name_of_the_subscriber = n.subscribe("topic_name")
        this->odom_sub = n.subscribe("amcl_pose", 10, &PathFollowingController::odomCallback, this);
        this->map_sub = n.subscribe("map_1",10,&PathFollowingController::mapCallback,this);

        this->n.getParam("/startX", startX);
        this->n.getParam("/startY", startY);
        this->n.getParam("/endX", endX);
        this->n.getParam("/endY", endY);
        this->n.getParam("/K_psi", K_psi);
        this->n.getParam("/K_omega", K_omega);
        this->n.getParam("/p", p);
        //this->n.getParam("/x_offset", x_offset);
        //this->n.getParam("/y_offset", y_offset);

    }

    void run(){

        // Send messages in a loop
        ros::Rate loop_rate(2);
        while (ros::ok())
        {
            if(path_set && pos_set){
              auto msg = calculateCommand();

              // Publish the new command
              this->path_follow_vel_pub.publish(msg);
            }

            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "path_following_controller");


    // Create our controller object and run it
    auto controller = PathFollowingController();
    controller.run();
}
