#ifndef MazeProblem_h__guard
#define MazeProblem_h__guard


#include "../Problem.h"
#include "MazeWorld.h"

struct MazeProblem : public Problem {
  
  //doesn't own world
  MazeProblem(const MazeWorld* world) : world(world){}
  
  double r(const State* s, const Action *a, const State *s_prime);
  
  bool isFinal(const State *s) const;
  
  bool solved() const;
  
  double getGamma() const {return 0.99;}
  
  const MazeWorld* world;

};


#endif
