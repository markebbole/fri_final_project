#ifndef State_h__guard
#define State_h__guard

#include <list>

class Action;
class State;

struct State {
  
  virtual bool lessThen(const State* other) const =0;
    
  virtual std::list<const Action*> availableActions() const = 0;
  
  virtual State *clone() const = 0;
  
  virtual ~State() {}
};


#endif
