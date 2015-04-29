#ifndef Agent_h__guard
#define Agent_h__guard

#include <iosfwd>

class Action;
class State;

struct Agent {
  
  virtual void act() = 0;
  
  virtual void readFrom(std::istream& stream) = 0;
  virtual void writeTo(std::ostream& stream) = 0;
  
  virtual ~Agent() {}
  
};


#endif
