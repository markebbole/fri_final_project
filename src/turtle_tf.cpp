
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "angles/angles.h"

#include "tf/transform_datatypes.h"

#include <cmath>

ros::Publisher chatter_pub;

double target_x = 2;
double target_y = 9;

void jill(const turtlesim::Pose::ConstPtr &jack){
   ROS_INFO_STREAM(*jack);
   
   tf::Transform transform;
   
   transform.setOrigin(tf::Vector3(jack->x, jack->y, 0));
   
   tf::Quaternion q;
   q.setRPY(0,0,jack->theta);
   transform.setRotation(q);
   
   tf::Transform usefulTransform = transform.inverse();
   
   tf::Vector3 pos =  usefulTransform(tf::Vector3(target_x, target_y, 0));
   
//    double x = target_x - jack->x;
//    double y = target_y - jack->y;
//    
//    double target_theta = atan2(y,x);
   
   geometry_msgs::Twist ball;
   
   ball.linear.x= pos.length();
   
   double target_theta = pos.angle(tf::Vector3(1,0,0));
   
   ball.angular.z =  -target_theta;
   
   
   chatter_pub.publish(ball);
  
}



int main(int argc, char** cc) {
   ros::init(argc, cc, "turtlecontroller");
   
   ros::NodeHandle n;
   ros::Subscriber ros_pose = n.subscribe("/turtle1/pose", 1000, jill);
   
   chatter_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
   ros::spin();
   
}