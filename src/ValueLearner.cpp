#include "ValueLearner.h"

#include "FourierFA.h"

#include "ros/console.h"


#include <valarray>

using namespace std;
using namespace pargo;

const double epsilon = 0.15;

const double max_velocity = 3.;
const double min_velocity = -3.;
const double ROBOT_SPEED = 0.26;
const double ROBOT_TURN_SPEED = 0.20;

vector<BoundsPair> extendBounds(const std::vector<BoundsPair> &bounds) {
  vector<BoundsPair> state_action_bounds(bounds.begin(),bounds.end());

  state_action_bounds.push_back(make_pair(-2*ROBOT_SPEED,2*ROBOT_SPEED));

  state_action_bounds.push_back(make_pair(-2*ROBOT_TURN_SPEED, 2*ROBOT_TURN_SPEED));
            
  return state_action_bounds;
}

ValueLearner::ValueLearner( const std::vector<BoundsPair> &bounds, unsigned int order) : 
          approx(new FourierFA(extendBounds(bounds), FullFourierCoefficientGenerator(extendBounds(bounds).size(),order))), 
          theta(approx->getNumBasisFunctions()),
          e(approx->getNumBasisFunctions()),
          gamma(0.99),
          lambda(0.9),
          alpha(0.05) {
  ROS_INFO_STREAM("The approximator has " << approx->getNumBasisFunctions() << " features");
}


geometry_msgs::Twist ValueLearner::computeAction(const std::vector<double>& state, Node* currentPosition) {
  //std::cout << "COMPUTING ACTION" << std::endl;
  //std::cout << state[0] << std::endl;
  vector<double> state_action(state.begin(), state.end());
  state_action.reserve(state_action.size() + 2);
  state_action.push_back(-ROBOT_SPEED); //initial action
  state_action.push_back(0.);
  double &linear = state_action[state_action.size() -2];
  double &angular = state_action[state_action.size() - 1];
  
  vector<double> theta_v(&theta[0],(&theta[0])+theta.size());
  
  geometry_msgs::Twist action;
  bool noMarkers = true;
  for(int i = 0; i < state.size()/2; i++) {
    if(state[i] != NULL) {
      noMarkers = false;
      break;
    }
  }
  std::cout << "are there no markers: " << noMarkers << std::endl;
  if(rand() <= epsilon * RAND_MAX) {
    //random action
    double v = rand();
    double v2 = rand();
    if((v / RAND_MAX) > .5 && !noMarkers) {
  		action.linear.x = (v2 / RAND_MAX) > .5 ? ROBOT_SPEED : -ROBOT_SPEED;
  	} else {
  		action.angular.z = (v2 / RAND_MAX) > .5 ? ROBOT_TURN_SPEED : -ROBOT_TURN_SPEED;
  	}
    
  }else {
    
    //best action
  
    action.linear.x = linear;
    action.angular.z = angular;
  

    double best_value = approx->value(theta_v,state_action);
  
    bool angularMove = false;
    stringstream values;
    double bestLinear = linear;
    //iterate through all actions, pick best one
    for(linear = -ROBOT_SPEED ;linear < ROBOT_SPEED+.1; linear += 2*ROBOT_SPEED) {
        double value = approx->value(theta_v,state_action);
        values << linear << " " << angular << ": " << value << " ";
        
        if(value > best_value ) {
          best_value = value;
          //action.linear.x = linear;
          bestLinear = linear;
        }
    }
    linear = 0.;
    if(noMarkers) {
      action.linear.x = 0.;
      linear = 0.;
      angular = -ROBOT_TURN_SPEED;
      best_value = approx->value(theta_v, state_action);
      angularMove = true;
    }

    for(angular = -ROBOT_TURN_SPEED ;angular < ROBOT_TURN_SPEED+.1; angular += 2*ROBOT_TURN_SPEED) {
        double value = approx->value(theta_v,state_action);
        values << linear << " " << angular << ": " << value << " ";
        
        if(value > best_value ) {
          best_value = value;
          action.angular.z= angular;
          action.linear.x = 0.;
          angularMove = true;
        }
    }
    
    if(!angularMove) {
		  action.linear.x = bestLinear;
		  action.angular.z = 0.;
	  }

    ROS_INFO_STREAM(values.str());
    ROS_INFO_STREAM("chosen action: " << action.linear.x << " " << action.angular.z);
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
  
  if(r == 100) { //reset to new episode
    for(int i = 0; i < e.size(); i++) {
		e[i] = 0.;
    }
  }
  
  
}
