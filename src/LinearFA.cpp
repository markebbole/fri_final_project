#include "LinearFA.h"

#include <numeric>

using namespace std;

namespace pargo {

double LinearFA::value(const std::vector<double>& parameters,const std::vector<double>& observations) {

    vector<double> features = computeFeatures ( observations );

    return inner_product(parameters.begin(),parameters.end(),features.begin(),0.);

}

LinearFA::~LinearFA() {}

}
