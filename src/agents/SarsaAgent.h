#ifndef SarsaAgent_h__guard
#define SarsaAgent_h__guard

#include "ValueObserver.h"
#include "../Agent.h"

#include <map>
#include <set>
#include <list>

class Problem;
class World;

struct StateComparator {
  bool operator()(const State *first, const State* second) const;
};

struct ActionComparator {
  bool operator()(const Action *first, const Action *second) const;
};

struct SarsaAgent : public Agent {
  
  struct Params {
    double alpha;
    double epsilon;
  };
  
  SarsaAgent(Problem *problem, World *world, const SarsaAgent::Params& params);
  
  void act();
  
  void readFrom(std::istream& stream);
  void writeTo(std::ostream& stream);
  
  typedef std::map<const Action*, double, ActionComparator> ActionMap;
  typedef std::map<const State*, ActionMap, StateComparator> StateActionMap;
  
  void addValueObserver(ValueObserver *obs);
  void removeValueObserver(ValueObserver *obs);
  
  ~SarsaAgent();
  
  private:
    
  const Action* chooseAction(const State *s);
  void learn(const State *, const Action*);
  
  Problem *problem;
  World *world;
  StateActionMap value;
  std::pair<const State *, const Action *> previous_pair;
  
  typedef std::set<const Action*, ActionComparator> ActionSet;
  ActionSet all_actions;
  
  const Action *my_action(const Action *other);
  
  double alpha;
  double epsilon;
  double gamma;
  
  std::list<ValueObserver*> observers;
};


#endif
