#ifndef Action_h__guard
#define Action_h__guard


struct Action {
  
  virtual bool lessThen(const Action *other) const =0;
  
  virtual Action *clone() const = 0;
  
  virtual ~Action() {}
  
};


#endif
