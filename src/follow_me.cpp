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

static tf::Vector3 transformVector;
float human_x, human_y, human_w, human_id;
float human_current_distance, detection_distance = 1.0, tracking_distance = 0.6;
float output_linear_v = 0.0, output_angular_velocity = 0.0;
float error_v=0, error_w=0, error_v_sum=0, error_w_sum=0;
float odom_v=0, odom_w=0;
float pre_v=0, pre_w=0;
float target_v=0, target_w=0;
float delta_v=0, delta_w=0;
float cmd_v=0, cmd_w=0;
/*
float min_dist=35.0;
float keep_range_in=40.0,keep_range_out=60.0;
float track_dist=min_dist+keep_range_in;
float max_dist=min_dist+keep_range_in+keep_range_out;
float max_dist_side=max_dist*0.7;
float limit_dist=150;
*/
float robot_distance, target_d_pre;
float robot_theta=0, target_th_pre=0;
float error_d=0, error_th=0;

void Tracking(){
  // PI Controller for linear velocity and angular velocity
  // Getting the distance between the robot and the human
  robot_distance = sqrt(pow(human_x, 2) + pow(human_y, 2));
  robot_theta = atan2(human_y,human_x);
  error_distance = robot_distance - tracking_distance;
  output_linear_v = Kp * (robot_distance - tracking_distance); //+ Ki * ()
}

void OdomCallBack(const nav_msgs::Odometry& msg){
  odom_v=msg.twist.twist.linear.x;
  odom_w=msg.twist.twist.angular.z;
  ROS_INFO("robot_odom_v %.2f robot_odom_w %.2f", odom_v, odom_w);
}

// Detection and tracking
void peopleTrackedCallBack(const leg_tracker::PersonArray::ConstPtr& personArray){
  int human_detected_number = personArray->people.size();
  if (human_detected_number != 0){
    ROS_INFO("human_detected_number: %d", human_detected_number);
    for (int i = 0; i < human_detected_number; i++){
      human_x = personArray->people[i].pose.position.x;
      human_y = personArray->people[i].pose.position.y;
      human_w = personArray->people[i].pose.orientation.w;
      human_id = personArray->people[i].id;
      //ROS_INFO("human_x : %.2f, human_y : %.2f, human_id : %d", human_x, human_y, human_id);
      //ROS_INFO("human_w : %.2f", human_w);
      //float distance = pow((x - CENTER_X), 2) + pow((y - CENTER_Y), 2);
      human_current_distance = sqrt(pow(human_x, 2) + pow(human_y, 2));
      ROS_INFO("human_x : %.2f, human_y : %.2f, human_id : %d, human_distance : %.2f", human_x, human_y, human_id, human_current_distance);
      if(human_current_distance <= detection_distance){
        ROS_INFO("Detected");
        if(human_current_distance <= tracking_distance){
          Tracking();
        }
      }
      else{
        ROS_INFO("Not Detected");
      }
    }
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "follow_me_node");
  ros::NodeHandle n;
  ros::Subscriber human_tracked_sub_, from_android_sub_, odom_sub_;
  ros::Publisher robot_status_pub_;
  
  human_tracked_sub_ = n.subscribe("/people_tracked", 100, peopleTrackedCallBack);
  from_android_sub_  = n.subscribe("/follow_me", 100, peopleTrackedCallBack);
  odom_sub_          = n.subscribe("/rugby/odom", 100, OdomCallBack);
  robot_status_pub_  = n.advertise<lidar_follow_me::RobotStatus>("follow_me_status", 10);
  ros::spin();
  return 0;
}