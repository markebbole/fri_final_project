#ifndef ValueObserver_h__guard
#define ValueObserver_h__guard

class State;

#include <vector>

struct ValueObserver {

  virtual void valueChanged(const State *s, const std::vector<double>& old_values, const std::vector<double>& new_values) = 0;
  
  virtual ~ValueObserver() {}
  
};

#endif
