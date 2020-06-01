#include "Header.h"

ros::NodeHandle nh;

//using rosserial_arduino::Test; 

//Publisher message definition
geometry_msgs::Pose2D pose_msg;
std_msgs::Float32 front_sensor_msg;
std_msgs::Float32 left_sensor_msg;
std_msgs::Float32 right_sensor_msg;

//Publisher Topics
ros::Publisher pose("pose", &pose_msg);
ros::Publisher front_distance("front_distance", &front_sensor_msg); 
ros::Publisher left_distance("left_distance", &left_sensor_msg); 
ros::Publisher right_distance("right_distance", &right_sensor_msg); 


//Callback Functions
void setVelocity(const geometry_msgs::Twist& vel_msg){
  float Vd_ = vel_msg.linear.x;
  float Wd_ = vel_msg.angular.z;
  if(Vd_ > 0.075 ){
    Vd_ = 0.075;
  }
  if(Vd_ < -0.075 ){
    Vd_ = -0.075;
  }
  if(Wd_ > 1.58 ){
    Wd_ = 1.58;
  }
  if(Wd_ < -1.58 ){
    Wd_ = -1.58;
  }
  Vd = Vd_; //[+/- 0.075 m/s]
  Wd = Wd_; //[+/- 1.58 rad/s]
  vel_Flag = true;
  
}

void setPos(const geometry_msgs::Pose2D& pos_set_msg){
  
  initial_pos.x = pos_set_msg.x;
  initial_pos.y = pos_set_msg.y;
  initial_pos.theta = pos_set_msg.theta;
  init_pos_Flag = true;
  
}

//Subscriber Topics
//ros::Subscriber<std_msgs::UInt8MultiArray> sub_leds("rgb_leds", setLED);
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("reactive_vel", setVelocity); 
//ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", setVelocity);
ros::Subscriber<geometry_msgs::Pose2D> sub_set_pose("set_pose", setPos);


void setup() {
   
  nh.getHardware()->setBaud(57600);

  //Initialise ROS serial node
  nh.initNode();

  //Subscriber Initilisation
  nh.subscribe(sub_set_pose);
  //nh.subscribe(sub_leds);
  nh.subscribe(sub_cmd_vel);
  
  //Advertising Publisher Topics (Initilisation)
  nh.advertise(pose);
  nh.advertise(front_distance);
  nh.advertise(left_distance);
  nh.advertise(right_distance);

  while(!nh.connected()){
    nh.spinOnce();
  }
  
  pinMode(PWMR,OUTPUT);
  pinMode(PWML,OUTPUT);
  pinMode(DIRR,OUTPUT);
  pinMode(DIRL,OUTPUT);


  Timer3.initialize(100000); // 100000 microseconds = 0.1 s
  Timer3.attachInterrupt(MotorSpeedControl); // Update speed when timer overlows (10Hz)  
  nh.spinOnce();

}

void MotorSpeedControl(void)
{   
    if(vel_Flag){
      encUpdate();
      poseUpdate();
      cmd_vel2wheel(Vd,Wd,&WLd, &WRd);
      cmd_vel2wheel(Vr,Wr,&WLr, &WRr);
      //ModelController(Vd, Wd, WLr, WRr);
      pid_controller1(Vd, Wd, WLr, WRr);
    } else{
      Vd = 0;
      Wd = 0;
    }
}

void loop() {
  while(nh.connected()){
    current_time=millis(); 
    
    // Start 10Hz loop
    if ((current_time-previous_time>= sampling_time)){
      previous_time=current_time;
      
      readSensors(&left_dis, &middle_dis, &right_dis);
      front_sensor_msg.data = middle_dis;
      left_sensor_msg.data = left_dis;
      right_sensor_msg.data = right_dis;
  
      geometry_msgs::Pose2D pose_MSG;
      pose_MSG.x = current_pos.x + initial_pos.x;
      pose_MSG.y = current_pos.y + initial_pos.y;
      pose_MSG.theta = current_pos.theta + initial_pos.theta;
  
      
  
      //Publish Topics
      pose.publish(&pose_MSG);
      front_distance.publish(&front_sensor_msg);
      left_distance.publish(&left_sensor_msg);
      right_distance.publish(&right_sensor_msg);
  
      
      //Spin node to process callbacks
      nh.spinOnce();
    }
  }
  // If ros is not connected, don't move
  Vd = 0.0;
  Wd = 0.0;
}

/*
  //Defining type of LED, fills array with LED objects
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);

  //Controls brightness of LEDs
  FastLED.setBrightness(255);
 */

/*
//Service server
ros::ServiceServer<Test::Request, Test::Response> buzzer_service("switch_buzzer_state",&switchBuzzerState);

//Advertising Service for Buzzer
nh.advertiseService(buzzer_service);

//Service callback function
void switchBuzzerState(const Test::Request & req, Test::Response & res){
  
  int buzzer_state = atoi(&req.input[0]);
  
  if(buzzer_state == 0){
  
    digitalWrite(BUZZER_PIN,LOW);
  
    res.output = "Buzzer OFF";
  
  }else if(buzzer_state == 1){
  
    digitalWrite(BUZZER_PIN,HIGH);
  
    res.output = "Buzzer ON";
  
  }else{
    res.output = "Unknown input command";
  }
}

*/


/*
//NOT SURE HOW AND IF THE COMPILER KNOWS THE LENGTH OF LED_msg, THIS MIGHT NOT COMPILE http://alexsleat.co.uk/2011/07/02/ros-publishing-and-subscribing-to-arrays/
void setLED(const std_msgs::UInt8MultiArray& LED_msg ){
  
  // LED_msg.data = array of uint8 [0-255]
  //Turn both LED's on - same colour
  leds[0] = CRGB(LED_msg[0],LED_msg[1],LED_msg[2]);
  FastLED.show(); 
  leds[1] = CRGB(LED_msg[3],LED_msg[4],LED_msg[5]);
  FastLED.show();
  
  
}
*/
