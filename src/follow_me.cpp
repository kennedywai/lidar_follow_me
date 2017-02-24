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

static tf::Vector3 transformVector;
float tracking_distance = 0.6;
float Kp, Ki;
float error_v=0, error_w=0, error_v_sum=0, error_w_sum=0;
float odom_v=0, odom_w=0;
float pre_v=0, pre_w=0;
float target_v=0, target_w=0;
float delta_v=0, delta_w=0;
float cmd_v=0, cmd_w=0;

float min_dist=35.0;
float keep_range_in=40.0,keep_range_out=60.0;
float track_dist=min_dist+keep_range_in;
float max_dist=min_dist+keep_range_in+keep_range_out;
float max_dist_side=max_dist*0.7;
float limit_dist=150;
float target_d,target_d_pre;
float target_th=0,target_th_pre=0;
float error_d=0,error_th=0;

void OdomCallBack(const nav_msgs::Odometry& msg){
  odom_v=msg.twist.twist.linear.x;
  odom_w=msg.twist.twist.angular.z;
  ROS_INFO("odom_v %f odom_w %f",odom_v,odom_w);
}

// Detection and tracking
void peopleTrackedCallBack(const leg_tracker::PersonArray::ConstPtr& personArray){
  int human_detected_number = personArray->people.size();
  if (human_detected_number != 0) {
    ROS_INFO("human_detected_number: %d", human_detected_number);
    for (int i = 0; i < human_detected_number; i++){
      float human_x = personArray->people[i].pose.position.x;
      float human_y = personArray->people[i].pose.position.y;
      int human_id = personArray->people[i].id;
      //ROS_INFO("x : %.2f, y : %.2f, id : %d", human_x, human_y, human_id);
      //float distance = pow((x - CENTER_X), 2) + pow((y - CENTER_Y), 2);
      float distance = sqrt(pow(human_x, 2) + pow(human_y, 2));
        ROS_INFO("x : %.2f, y : %.2f, id : %d distance : %.2f", human_x, human_y, human_id, distance);
      if(distance <= tracking_distance){
        ROS_INFO("Detected");
      }else{
        ROS_INFO("Not Detected");
        }
    }
  }
}

// PI Controller for linear velocity and angular velocity
void pi_controller(){

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