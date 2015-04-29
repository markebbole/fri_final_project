#ifndef FileValueObserver_h__guard
#define FileValueObserver_h__guard

#include "ValueObserver.h"


#include <fstream>

class State;

struct FileValueObserver : public ValueObserver {
  
  FileValueObserver(const std::string& fileName, State *desired_state);
  
  void valueChanged(const State *s, const std::vector<double>& old_values, const std::vector<double>& new_values);
  
  ~FileValueObserver();
private:
  
  std::ofstream file;
  State *desired_state;
  
};

#endif