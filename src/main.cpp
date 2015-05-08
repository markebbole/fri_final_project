
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "angles/angles.h"
#include "LearningController.h"
#include "ValueLearner.h"

#include "std_srvs/Empty.h"
#include "turtle_controller/TargetPose.h"

#include <cmath>

using namespace std;

ros::Publisher velocity_pub;

ros::ServiceClient client;
LearningController *controller;

double target_x = 8;
double target_y = 2;
double target_v = 0;

vector<double> lastState;
geometry_msgs::Twist lastAction;
const turtlesim::Pose::ConstPtr &lastPose;
geometry_msgs:
double rate = 5;
ros::Time last_command;

double ball_x = 7.;
double ball_y = 4.;

double r(double poseX, double poseY, const vector<double>& s, geometry_msgs::Twist& a,  const vector<double>& s_prime) {
  


  double ball_distance_x = poseX - ball_x;
  double ball_distance_y = poseY - ball_y;
  double ball_distance = sqrt(ball_distance_x * ball_distance_x + ball_distance_y * ball_distance_y);

  /*if(ball_distance < 1) {
    ROS_INFO_STREAM("HIT BARRIER: REWARD = -2000");
    return -2000;
  }*/
  //first approximation of the reward: distance to the target
  
  double reward;
  
  if(s_prime[0] < 0.5) {
    reward = 100;
  else
    reward = -1;//-s_prime[0];
    
  
  ROS_INFO_STREAM("reward: " << reward << " " << target_x << " " << target_y);

  return reward;
  
//   double x = s_prime[0] - target_x;
//   double y = s_prime[1] - target_y;
//   double v = s_prime[2] - target_v;
//   
//   return -sqrt(x*x + y*y);
}

void reset() { 
  std_srvs::Empty c;
  client.call(c);
  //target_x = (rand() % 4) + 3;
  //target_y = (rand() % 4) + 3;
}

void receivePose(const turtlesim::Pose::ConstPtr &pose){
  
  if((ros::Time::now() - last_command).toSec() < 1./rate)
    return; //wait for the effect of our action
    
   double diff_x = target_x - pose->x;
   double diff_y = target_y - pose->y;
  
   vector<double> state(2,0.);
   state[0] = sqrt(diff_x * diff_x + diff_y * diff_y);
  // state[1] = target_v - pose->linear_velocity;
   
   double angle = atan2(diff_y,diff_x);
   
   state[1] = angles::shortest_angular_distance(pose->theta,angle);
   
   
   geometry_msgs::Twist action = controller->computeAction(state);
   
   if(!lastState.empty()) {
     double reward = r(pose->x, pose->y, lastState,lastAction,state);
     controller->learn(lastState,lastAction,reward,state, action);
     if(reward == 100) {
       lastState.clear();
       reset();
     }
   }
   
   lastState = state;
   lastAction = action;
   
   
   last_command = ros::Time::now();
   velocity_pub.publish(action);
  
}


bool setTarget(turtle_controller::TargetPose::Request &req, turtle_controller::TargetPose::Response &res) {
   target_x = req.x;
   target_y = req.y;
   target_v = req.v;

   return true;
}




int main(int argc, char** cc) {
  
   srand(time(NULL));
   
   ros::init(argc, cc, "turtlecontroller");
   
   vector<pargo::BoundsPair> bounds;
   bounds.push_back(make_pair(-5.,50.));
  // bounds.push_back(make_pair(-15.,15.));
   bounds.push_back(make_pair(-M_PI,M_PI));
   
   controller = new ValueLearner(bounds,5);
   
   ros::NodeHandle n;
   ros::Subscriber ros_pose = n.subscribe("/turtle1/pose", 1000, receivePose);
   client = n.serviceClient<std_srvs::Empty>("reset");
   
   ros::ServiceServer service = n.advertiseService("target_pose", setTarget);
   
   velocity_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
   
   last_command = ros::Time::now(); //initialize the variable

   ros::spin();
   
   delete controller;
   
}
