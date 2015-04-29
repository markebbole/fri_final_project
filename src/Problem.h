#ifndef Problem_h__guard
#define Problem_h__guard

class State;
class Action;

struct Problem {
  
  virtual double r(const State* s, const Action *a, const State *s_prime) = 0;
  
  virtual bool solved() const =0;
  
  virtual bool isFinal(const State *s) const = 0;
  
  virtual double getGamma() const = 0;

  virtual ~Problem() {}
};

#endif

