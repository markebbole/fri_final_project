#ifndef ValueLearner_h___guard
#define ValueLearner_h___guard

#include "LearningController.h"
#include "LinearFA.h"
#include "BoundsPair.h"
#include <valarray>

struct ValueLearner : public LearningController{
  
  ValueLearner( const std::vector<pargo::BoundsPair> &bounds, unsigned int order);
  
  geometry_msgs::Twist computeAction(const std::vector<double>& state); 
  
  void learn(const std::vector<double>& s,const geometry_msgs::Twist& a , double r, 
                                  const std::vector<double>&s_prime,const geometry_msgs::Twist& a_prime );
  void resetEligibilityTrace();
private:
  pargo::LinearFA *approx;
  std::valarray<double> theta;
  std::valarray<double> e;
  double gamma;
  double lambda;
  double alpha;
};

#endif
