#include "ros/ros.h"
#include "math.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "sys/time.h"
#include "iostream"
#include "stdio.h"
#include "stdlib.h"
#include "leg_tracker/Person.h"
#include "leg_tracker/PersonArray.h"
#include "lidar_follow_me/RobotStatus.h"
#include "tf/transform_listener.h"

#define Kp 0.4
#define Ki 0.02
#define MAX_SPEED 0.3

static tf::Vector3 transformVector;
ros::Subscriber human_tracked_sub_, from_android_sub_, odom_sub_;
ros::Publisher robot_status_pub_, command_robot_pub_;
int   human_id;
bool  detected = false;
float human_x, human_y, human_w;	
float human_current_distance, detection_distance=1.0, tracking_distance=0.7;
float error_distance, error_distance_i=0;
float output_linear_v = 0.0, output_angular_velocity = 0.0;
float error_v=0, error_w=0, error_v_sum=0, error_w_sum=0;
float odom_v=0, odom_w=0;
float robot_human_distance, target_d_pre;
float robot_theta=0, target_th_pre=0;
float error_d=0, error_th=0;

void Tracking(){
  // PI Controller for linear velocity and angular velocity
  // Getting the distance between the robot and the human
  geometry_msgs::Twist command_robot;
  robot_human_distance = sqrt(pow(human_x, 2) + pow(human_y, 2));
  robot_theta = atan2(human_y,human_x);

  error_distance = robot_human_distance - tracking_distance;
  error_distance_i = error_distance_i + error_distance;
  output_linear_v = Kp * (error_distance) + Ki * (error_distance_i);
  if (output_linear_v >= MAX_SPEED) 
    output_linear_v = MAX_SPEED;
  else if (output_linear_v <= -1*MAX_SPEED) 
    output_linear_v = -1*MAX_SPEED;
  ROS_INFO("output_linear_v : %.2f", output_linear_v);
  command_robot.linear.x = output_linear_v;
  command_robot_pub_.publish(command_robot);
}

void Stop(){
  geometry_msgs::Twist command_robot;
  detected = false;
  command_robot.linear.x = 0.0;
  command_robot_pub_.publish(command_robot);	
}

void OdomCallBack(const nav_msgs::Odometry& msg){
  odom_v = msg.twist.twist.linear.x;
  odom_w = msg.twist.twist.angular.z;
  //ROS_INFO("robot_odom_v : %.2f robot_odom_w : %.2f", odom_v, odom_w);
}

// Detection and tracking
void peopleTrackedCallBack(const leg_tracker::PersonArray::ConstPtr& personArray){
  geometry_msgs::Twist command_robot;
  int human_detected_number = personArray->people.size();
  if (human_detected_number != 0){
    ROS_INFO("human_detected_number : %d", human_detected_number);
    for (int i = 0; i < human_detected_number; i++){
      human_x  = personArray->people[i].pose.position.x;
      human_y  = personArray->people[i].pose.position.y;
      human_w  = personArray->people[i].pose.orientation.w;
      human_id = personArray->people[i].id;
      human_current_distance = sqrt(pow(human_x, 2) + pow(human_y, 2));
      ROS_INFO("human_id : %d", human_id);
      //ROS_INFO("human_x : %.2f, human_y : %.2f", human_x, human_y);
      ROS_INFO("human_distance : %.2f", human_current_distance);
      //ROS_INFO("human_w : %.2f", human_w);
      detected = true;
      Tracking();
      //float distance = pow((x - CENTER_X), 2) + pow((y - CENTER_Y), 2);
    }
  }
  else
    detected = false;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "follow_me_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  command_robot_pub_ = n.advertise<geometry_msgs::Twist>("/rugby/cmd_vel", 10);
  robot_status_pub_  = n.advertise<lidar_follow_me::RobotStatus>("follow_me_status", 10);
  human_tracked_sub_ = n.subscribe("/people_tracked", 100, peopleTrackedCallBack);
  from_android_sub_  = n.subscribe("/follow_me", 100, peopleTrackedCallBack);
  odom_sub_          = n.subscribe("/rugby/odom", 100, OdomCallBack);

  while(ros::ok()){
  if(!detected){
    ROS_INFO("NOT DETECTED");  
    Stop();
    }
  ros::spinOnce();
  loop_rate.sleep();
  }
  return 0;
}