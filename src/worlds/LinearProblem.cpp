#include "LinearProblem.h"

#include <iostream>

bool LinearProblem::solved() const {
  const LinearWorldState *s = world->getCurrentState();
  
  bool done = isFinal(s);
  delete s;
  
  return done;
}

bool LinearProblem::isFinal(const State *s) const {
  const LinearWorldState *s_linear = dynamic_cast<const LinearWorldState *>(s);
  
  if(s_linear == NULL)
    std::cerr << "LinearProblem: state not from LinearWorld" << std::endl;
  
  return s_linear->pos == world->getMaxPos();
}

double LinearProblem::r(const State* s, const Action *a, const State *s_prime) {
    if(isFinal(s_prime))
      return 100.;
    
    return 0.;
}