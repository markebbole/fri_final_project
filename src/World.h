#ifndef World_h__guard
#define World_h__guard

class State;
class Action;

struct World {
  
  virtual State* getCurrentState() const = 0;
  
  virtual void applyAction(const Action *a) = 0;
  
  virtual void reset() = 0;
  
  virtual ~World() {}
  
};

#endif
