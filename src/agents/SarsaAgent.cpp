#include "SarsaAgent.h"

#include "../State.h"
#include "../Action.h"
#include "../Problem.h"
#include "../World.h"

#include <list>
#include <algorithm>
#include <iterator>
#include <iostream>
using namespace std;

bool StateComparator::operator()(const State *first, const State* second) const {
    return first->lessThen(second);
}

bool ActionComparator::operator()(const Action *first, const Action *second) const {
    return first->lessThen(second);
}

struct CompareSecond {
    bool operator()(const pair<const Action *, double>& p1, const pair<const Action*, double> p2) const {
        return p1.second < p2.second;
    }
};

struct RetriveActionValue {

    RetriveActionValue(const SarsaAgent::ActionMap &value) : value(value) {}

    pair<const Action*, double> operator()(const Action * act) const {
        SarsaAgent::ActionMap::const_iterator action_val = value.find(act);
        if(action_val != value.end())
            return *action_val;
        else
            return make_pair(act,0.); // return default value if this action has never been executed before
    }

    const SarsaAgent::ActionMap &value;
};

SarsaAgent::SarsaAgent(Problem *problem, World *world, const SarsaAgent::Params& p) :
    problem(problem),
    world(world),
    value(),
    previous_pair(NULL,NULL),
    all_actions(),
    alpha(p.alpha),
    epsilon(p.epsilon),
    gamma(problem->getGamma()) {}

void SarsaAgent::act() {
    State *s = world->getCurrentState();

    const Action *a = this->chooseAction(s);

    world->applyAction(a);

    if(previous_pair.first != NULL)
      this->learn(s,a);
    
    delete previous_pair.first;
    previous_pair = make_pair(s,a);
    
    if(problem->solved()) { //collect reward from last transition
      State *s_prime = world->getCurrentState();
      this->learn(s_prime,chooseAction(s_prime));
      delete s_prime;
      
      //clear memory, episode ended
      delete previous_pair.first;
      previous_pair.first = NULL;
      previous_pair.second = NULL;
    }

}


const Action* SarsaAgent::chooseAction(const State* s) {


    list<const Action*> actions =  s->availableActions();

    //with a small probability, take a random action
    if(rand() < epsilon * RAND_MAX) {
        int chosen_index = rand() % actions.size();
        list<const Action*>::const_iterator chosen = actions.begin();
        advance(chosen,chosen_index);

        return my_action(*chosen);
    }


    //If we got here, we want to return the action with the highest expected long-term reward.
    //In order to do that, we need to compute the value of each available action:

    list<pair<const Action*, double> > action_values;

    StateActionMap::const_iterator state_value = value.find(s);
    const ActionMap &map_to_use = (state_value != value.end())? (state_value->second) : ActionMap();

    transform(actions.begin(),actions.end(),back_inserter(action_values),RetriveActionValue(map_to_use));

    //Then, make a decision on that value:

    list<pair<const Action*, double> >::const_iterator  best =
        max_element(action_values.begin(), action_values.end(),CompareSecond());

    return my_action(best->first); //best->first is an Action*
}

const Action *SarsaAgent::my_action(const Action *other) {
    //do I have this action yet?
    ActionSet::const_iterator found_act = all_actions.find(other);
    if(found_act == all_actions.end()) //it's a new action! Add it
        found_act = all_actions.insert(other->clone()).first;
    
  return *found_act;
}

void SarsaAgent::readFrom(std::istream& stream) {}
void SarsaAgent::writeTo(std::ostream& stream) {}

struct ValueFromPair {
  
  ValueFromPair(const RetriveActionValue& f) : f(f) {}
  
  double operator()(const Action * act) {
    return f(act).second;
  }
  
  RetriveActionValue f;
};
  
struct NotifyOfChange {
  
  NotifyOfChange(const State *s, const vector<double> &v_before, const vector<double> &v_after) :
        s(s),
        v_before(v_before),
        v_after(v_after) {}
  
  void operator()(ValueObserver *v) {
   v->valueChanged(s,v_before,v_after); 
  }
  
  const State *s;
  const vector<double> &v_before;
  const vector<double> &v_after;
};

void SarsaAgent::learn(const State *s_prime, const Action*a_prime) {
  
    const State *s = previous_pair.first;
    const Action *a = previous_pair.second; //better names

    //it's important we don't store in the map pointers to states passed to this function,
    //because those will be destroyed.
    
    if(value.find(s) == value.end()) //s is not in the map
        value.insert(make_pair(s->clone(),ActionMap())); //insert an empty map for this (new) state

    //same thing for s_prime
    if(value.find(s_prime) == value.end()) //s is not in the map
        value.insert(make_pair(s_prime->clone(),ActionMap())); //insert an empty map for this (new) state
 

    double reward = problem->r(s,a,s_prime);
    
    list<const Action*> all_actions = s->availableActions();
    
    vector<double> action_values_before;
    transform(all_actions.begin(),all_actions.end(),back_inserter(action_values_before),ValueFromPair(RetriveActionValue(value[s])));
    
    //ready! Apply the algorithm:
    if(problem->isFinal(s_prime))
      value[s][a] += alpha * (reward - value[s][a]);
    else
      value[s][a] += alpha * (reward + gamma * value[s_prime][a_prime] - value[s][a]);
    
    vector<double> action_values_after;
    transform(all_actions.begin(),all_actions.end(),back_inserter(action_values_after),ValueFromPair(RetriveActionValue(value[s])));
    
    for_each(observers.begin(),observers.end(),NotifyOfChange(s,action_values_before,action_values_after));

}

void SarsaAgent::addValueObserver(ValueObserver *obs) {
 observers.push_back(obs);   
}
  
void SarsaAgent::removeValueObserver(ValueObserver *obs) {
  list<ValueObserver*>::iterator ob = find(observers.begin(), observers.end(), obs);
  if(ob != observers.end())
    observers.erase(ob);
}

SarsaAgent::~SarsaAgent() {
    //empty the map

    StateActionMap::iterator s_a_m = value.begin();
    for(; s_a_m != value.end(); ++s_a_m)
        delete s_a_m->first;
    
    ActionSet::iterator act = all_actions.begin();

    for(; act != all_actions.end(); ++act)
      delete *act;
    
    delete previous_pair.first;
      
}

