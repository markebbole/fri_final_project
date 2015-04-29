#ifndef LinearWorld_h__guard
#define LinearWorld_h__guard

#include "../World.h"
#include "../State.h"
#include "../Action.h"

#include <string>

struct LinearWorldState : public State{
  
  LinearWorldState(unsigned int pos, const std::list<const Action*>& actions) : pos(pos), actions(actions) {}
  
  unsigned int pos;
  
  bool lessThen(const State* other) const;   
  std::list<const Action*> availableActions() const;
  
  LinearWorldState *clone() const;
  
private:
  const std::list<const Action*> &actions;
};

struct LinearWorldAction : public Action{
  explicit LinearWorldAction(bool move_forward) : movement((move_forward)?1: -1) {}

  int getMovement()  const { return movement;}
  bool lessThen(const Action *other) const;
  Action *clone() const { return new LinearWorldAction(*this);}
  
  operator std::string() const {return (movement > 0)? std::string("forward") : std::string("backward");} 
 
private:
  int movement;
};


struct LinearWorld : public World {
  
  explicit LinearWorld(unsigned int max_position);

  LinearWorldState* getCurrentState() const;
  
  void applyAction(const Action *a);
  
  void reset();
  
  unsigned int getMaxPos() const {return max_pos;}
  
  ~LinearWorld();
  
  private:
  unsigned int max_pos;
  std::list<const Action*> actions;
  LinearWorldState currentState;
  
};




#endif
