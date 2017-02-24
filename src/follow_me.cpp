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

static float RANGE = 0.5 * 0.5;
static float CENTER_X = 1.45;
static float CENTER_Y = 0;

void publishTopic() {
  ros::NodeHandle n;
  ros::Publisher detection_pub = n.advertise<std_msgs::String>("follow_me_status", 100);
  if (ros::ok()) {
    std_msgs::String msg;
    msg.data = "DETECTED";
    detection_pub.publish(msg);
  }
}

void peopleTrackedCallback(const leg_tracker::PersonArray::ConstPtr& personArray){
  int size = personArray->people.size();

  if (size != 0) {
    ROS_INFO("size %d", size);
    for (int i = 0; i < size; i++) {
      float human_x = personArray->people[i].pose.position.x;
      float human_y = personArray->people[i].pose.position.y;
      int human_id = personArray->people[i].id;
      ROS_INFO("x : %.2f, y : %.2f, id : %d", human_x, human_y, human_id);
      //float distance = pow((x - CENTER_X), 2) + pow((y - CENTER_Y), 2);
      float distance = pow(human_x, 2) + pow(human_y, 2);
      if (distance <= RANGE) {
        ROS_INFO("Detected");
        publishTopic();
      } else {
        ROS_INFO("Not Detected");
      }
    }

  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("people_tracked", 100, peopleTrackedCallback);

  ros::spin();

  return 0;
}