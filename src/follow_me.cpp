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

#define Kp_linear  0.4
#define Ki_linear  0.02
#define Kp_angular 1.3
#define Ki_angular 0
#define MAX_SPEED_LINEAR  0.35
#define MAX_SPEED_ANGULAR  0.75

static tf::Vector3 transformVector;;
ros::Subscriber human_tracked_sub_, from_android_sub_, odom_sub_;
ros::Publisher robot_status_pub_, command_robot_pub_;
int   human_id, human_id_pre;
bool  detected = false;
float human_x, human_y, human_w;
float COS_THETA = 0, SIN_THETA = 0;
float human_current_distance, detection_distance=1.0, tracking_distance=0.75;
float error_distance, error_distance_i=0;
float output_linear_v = 0.0, output_angular_v = 0.0;
float error_linear_v = 0, error_angular_v = 0, error_linear_v_sum = 0, error_angular_v_sum = 0;
float odom_v = 0, odom_w = 0;
float robot_human_distance, target_d_pre;
float robot_theta = 0;

void Tracking(){
  // PI Controller for linear velocity and P Controller for angular velocity
  // Getting the distance between the robot and the human
  geometry_msgs::Twist command_robot;
  robot_human_distance = sqrt(pow(human_x, 2) + pow(human_y, 2));
  robot_theta = atan2(human_y,human_x);

  ROS_INFO("robot_theta : %.2f", robot_theta);
  ROS_INFO("human_x : %.2f, human_y : %.2f", human_x, human_y);
  ROS_INFO("human_w : %.2f human_theta : %.2f ", human_w, 2*acos(human_w));
  
  error_distance = robot_human_distance - tracking_distance;
  error_distance_i = error_distance_i + error_distance;
  
  //output_linear_v = Kp_linear * (error_distance) + Ki_linear * (error_distance_i);
  output_linear_v = Kp_linear * (error_distance);
  output_angular_v = Kp_angular * (robot_theta);

  if (output_linear_v >= MAX_SPEED_LINEAR) 
    output_linear_v = MAX_SPEED_LINEAR;
  else if (output_linear_v <= -1*MAX_SPEED_LINEAR) 
    output_linear_v = -1*MAX_SPEED_LINEAR;
  ROS_INFO("output_linear_v : %.2f", output_linear_v);

  if (output_angular_v >= MAX_SPEED_ANGULAR) 
    output_angular_v = MAX_SPEED_ANGULAR;
  else if (output_angular_v <= -1*MAX_SPEED_ANGULAR) 
    output_angular_v = -1*MAX_SPEED_ANGULAR;
  ROS_INFO("output_angular_v : %.2f", output_angular_v);

  command_robot.linear.x  = output_linear_v;
  command_robot.angular.z = output_angular_v;
  command_robot_pub_.publish(command_robot);
}

void Stop(){
  geometry_msgs::Twist command_robot;
  detected = false;
  command_robot.linear.x  = 0.0;
  command_robot.angular.z = 0.0;
  command_robot_pub_.publish(command_robot);	
}

void OdomCallBack(const nav_msgs::Odometry& msg){
  odom_v = msg.twist.twist.linear.x; // linear velocity
  odom_w = msg.twist.twist.angular.z;// angular velocity
  //ROS_INFO("robot_odom_v : %.2f robot_odom_w : %.2f", odom_v, odom_w);
}

// Detection and tracking
void peopleTrackedCallBack(const leg_tracker::PersonArray::ConstPtr& personArray){
  int human_detected_number = personArray->people.size();
  if (human_detected_number != 0){
    ROS_INFO("human_detected_number : %d", human_detected_number);
    for (int i = 0; i < human_detected_number; i++){
      human_x = COS_THETA * (personArray->people[i].pose.position.x) - SIN_THETA * (personArray->people[i].pose.position.y) + transformVector.getX();
      human_y = SIN_THETA * (personArray->people[i].pose.position.x) + COS_THETA * (personArray->people[i].pose.position.y) + transformVector.getY();
      human_w  = personArray->people[i].pose.orientation.w;
      human_id = personArray->people[i].id;
      detected = true;
      Tracking();
    }
  }
  else
    detected = false;	
}

int main(int argc, char **argv){
  ros::init(argc, argv, "follow_me_node");
  ros::NodeHandle n;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  ros::Rate loop_rate(10);	

  command_robot_pub_ = n.advertise<geometry_msgs::Twist>("/rugby/cmd_vel", 10);
  robot_status_pub_  = n.advertise<lidar_follow_me::RobotStatus>("/follow_me_status", 10);
  human_tracked_sub_ = n.subscribe("/people_tracked", 100, peopleTrackedCallBack);
  from_android_sub_  = n.subscribe("/follow_me", 100, peopleTrackedCallBack);
  odom_sub_          = n.subscribe("/rugby/odom", 100, OdomCallBack);
  while(ros::ok()){
  try {
    listener.waitForTransform("rugby_base", "rugby_rplidar", ros::Time(), ros::Duration(3.0));
      listener.lookupTransform("rugby_base", "rugby_rplidar", ros::Time(), transform);
      transformVector = transform.getOrigin();
      float x = transformVector.getX();
      float y = transformVector.getY();
      double yaw, pitch, roll;
      transform.getBasis().getRPY(roll, pitch, yaw);
      //ROS_INFO("x : %.2f, y : %.2f",x , y);
      //ROS_INFO("yaw : %.2f",yaw);
      COS_THETA = cos(yaw);
      SIN_THETA = sin(yaw);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return 0;
  }

  //while(ros::ok()){
  if(!detected){
    //ROS_INFO("NOT DETECTED");  
    Stop();
    }
  ros::spinOnce();
  loop_rate.sleep();
  }

  //ros::spin();
  return 0;
}
