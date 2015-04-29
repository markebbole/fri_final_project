#ifndef LinearProblem_h__guard
#define LinearProblem_h__guard


#include "../Problem.h"
#include "LinearWorld.h"

struct LinearProblem : public Problem {
  
  //doesn't own world
  LinearProblem(const LinearWorld* world) : world(world){}
  
  double r(const State* s, const Action *a, const State *s_prime);
  
  bool isFinal(const State *s) const;
  
  bool solved() const;
  
  double getGamma() const {return 0.99;}
  
  const LinearWorld* world;

};


#endif
