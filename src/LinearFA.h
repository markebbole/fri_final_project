#ifndef pargo_LinearFA_h__guard
#define pargo_LinearFA_h__guard

#include <vector>

namespace pargo {

struct LinearFA {

	virtual unsigned int getNumBasisFunctions() const = 0;
	
	virtual double value(const std::vector<double>& parameters,const std::vector<double>& observations);
	
	virtual std::vector<double> computeFeatures(const std::vector<double>& observations) const = 0;

	virtual ~LinearFA();


};

}

#endif
