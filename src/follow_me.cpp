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
float error_v, error_w;

// Detection and tracking
void peopleTrackedCallback(const leg_tracker::PersonArray::ConstPtr& personArray){
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
  ros::Subscriber human_tracked_sub_, from_android_sub_;
  ros::Publisher robot_status_pub_;
  /*
  tf::TransformListener listener;
  tf::StampedTransform transform;
  
  try {
    listener.waitForTransform("rugby_rplidar", "rugby_base", ros::Time(), ros::Duration(3.0));
      listener.lookupTransform("rugby_rplidar", "rugby_base", ros::Time(), transform);
      transformVector = transform.getOrigin();
      float x = transformVector.getX();
      float y = transformVector.getY();
      double yaw, pitch, roll;
      transform.getBasis().getRPY(roll, pitch, yaw);
      ROS_INFO("x: %f, y: %f",x , y);
      ROS_INFO("roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return 0;
  }
  */
  human_tracked_sub_ = n.subscribe("people_tracked", 100, peopleTrackedCallback);
  from_android_sub_  = n.subscribe("follow_me", 100, peopleTrackedCallback);
  robot_status_pub_  = n.advertise<lidar_follow_me::RobotStatus>("follow_me_status", 10);
  ros::spin();
  return 0;
}