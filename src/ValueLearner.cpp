#include "ValueLearner.h"

#include "FourierFA.h"

#include "ros/console.h"


#include <valarray>

using namespace std;
using namespace pargo;

const double epsilon = 0.05;

const double max_velocity = 1.;
const double min_velocity = -1.;
const double ROBOT_LINEAR_SPEED = 0.2;
const double ROBOT_ANGULAR_SPEED = 0.3;

vector<BoundsPair> extendBounds(const std::vector<BoundsPair> &bounds) {
  vector<BoundsPair> state_action_bounds(bounds.begin(),bounds.end());
  state_action_bounds.push_back(make_pair(min_velocity,max_velocity));
  state_action_bounds.push_back(make_pair(min_velocity,max_velocity));
            
  return state_action_bounds;
}

ValueLearner::ValueLearner( const std::vector<BoundsPair> &bounds, unsigned int order) : 
          approx(new FourierFA(extendBounds(bounds), FullFourierCoefficientGenerator(extendBounds(bounds).size(),order))), 
          theta(approx->getNumBasisFunctions()),
          e(approx->getNumBasisFunctions()),
          gamma(0.998),
          lambda(0.9),
          alpha(0.001) {
  ROS_INFO_STREAM("The approximator has " << approx->getNumBasisFunctions() << " features");
}


geometry_msgs::Twist ValueLearner::computeAction(const std::vector<double>& state) {
  //std::cout << "COMPUTING ACTION" << std::endl;
  //std::cout << state[0] << std::endl;
  vector<double> state_action(state.begin(), state.end());
  state_action.reserve(state_action.size() + 1);
  state_action.push_back(-ROBOT_LINEAR_SPEED); //initial action
  state_action.push_back(-ROBOT_ANGULAR_SPEED);
  double &linear = state_action[state_action.size() -2];
  double &angular = state_action[state_action.size()-1];
  
  vector<double> theta_v(&theta[0],(&theta[0])+theta.size());
  
  geometry_msgs::Twist action;
  
  if(rand() <= epsilon * RAND_MAX) {
    //random action
    double v = rand();
    double v2 = rand();
    action.linear.x = (v / RAND_MAX) > .5 ? ROBOT_LINEAR_SPEED : -ROBOT_LINEAR_SPEED;
    action.angular.z = (v2 / RAND_MAX) > .5 ? ROBOT_ANGULAR_SPEED : -ROBOT_ANGULAR_SPEED;
  }else {
    
    //best action
  
    action.linear.x = linear;
    action.angular.z = angular;
  
    double best_value = approx->value(theta_v,state_action);
  
  
    stringstream values;
    //iterate through all actions, pick best one
    for(angular = -ROBOT_ANGULAR_SPEED; angular < ROBOT_ANGULAR_SPEED + .1; angular += .1;)
      for(linear = -ROBOT_LINEAR_SPEED ;linear < ROBOT_LINEAR_SPEED+.1; linear += 2*ROBOT_LINEAR_SPEED) {
          double value = approx->value(theta_v,state_action);
          values << linear << " " << /*angular <<*/ ": " << value << " ";
          if(value > best_value ) {
            best_value = value;
            action.linear.x = linear;
          }
      }
    }
    ROS_INFO_STREAM("chosen action: " << action.linear.x);
    ROS_INFO_STREAM("action value: " << best_value);
  }
  
  return action;
  
}

void ValueLearner::learn(const vector<double>& s,const geometry_msgs::Twist& a , double r, 
                                  const vector<double>&s_prime,const geometry_msgs::Twist& a_prime ) {
  
  vector<double> state_action(s.begin(),s.end());
  state_action.push_back(a.linear.x);
  state_action.push_back(a.angular.z);
  
//   stringstream print_state_act;
//   copy(state_action.begin(),state_action.end(),ostream_iterator<double>(print_state_act, " "));
//   ROS_INFO_STREAM("state-action: " << print_state_act.str());
  
  
  vector<double> state_action_prime(s_prime.begin(),s_prime.end());
  state_action_prime.push_back(a_prime.linear.x);
  state_action_prime.push_back(a_prime.angular.z);
  
  vector<double> phi_s = approx->computeFeatures(state_action);
//     stringstream print_phi_s;
//   copy(phi_s.begin(),phi_s.end(),ostream_iterator<double>(print_phi_s, " "));
//   ROS_INFO_STREAM("phi_s: " << print_phi_s.str());
  
  valarray<double> phi_s_v(&phi_s[0],phi_s.size());

  vector<double> theta_v(&theta[0],(&theta[0])+theta.size());
  
  double v_hat_s = approx->value(theta_v,state_action);
  
  ROS_INFO_STREAM ("v_hat_s: " << v_hat_s);
  
  double v_hat_s_prime = approx->value(theta_v,state_action_prime);
  
  double delta = r + gamma * v_hat_s_prime - v_hat_s;
  
  e = gamma * lambda * e + alpha * (1 - gamma * lambda * (e * phi_s_v).sum()) * phi_s_v;
  
  
  theta = theta + delta * e + alpha * (v_hat_s - (theta * phi_s_v).sum()) * phi_s_v;
  
  
}
