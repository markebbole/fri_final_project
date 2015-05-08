#ifndef pargo_FourierFA_h__guard
#define pargo_FourierFA_h__guard

#include "BoundsPair.h"
#include "LinearFA.h"

#include <vector>
#include <valarray>

namespace pargo {

struct FourierCoefficientGenerator;

struct FourierFA : public LinearFA {

	FourierFA ( const std::vector<BoundsPair> &bounds, const FourierCoefficientGenerator& );

	unsigned int getNumBasisFunctions() const {
		return nterms;
	}
	
	std::vector<double> computeFeatures ( const std::vector<double>& observations ) const;


private:

	std::valarray<double> lb,ub;
	std::valarray<double> multipliers;
	unsigned int nterms;
};



struct FourierCoefficientGenerator {

	virtual std::valarray<double> allCoefficients() const = 0;
	virtual unsigned int numTerms() const = 0;

	virtual ~FourierCoefficientGenerator() {};
};


struct FullFourierCoefficientGenerator : public FourierCoefficientGenerator {

	FullFourierCoefficientGenerator ( unsigned int nvars,unsigned  int order ) :
		nvars ( nvars ),order ( order ) {}

	std::valarray<double> allCoefficients() const;
	unsigned int numTerms() const;

private:
	unsigned int nvars, order;
};

struct IndependentFourierCoefficientGenerator : public FourierCoefficientGenerator {

	IndependentFourierCoefficientGenerator ( unsigned int nvars,unsigned  int order ) :
		nvars ( nvars ),order ( order ) {}

	std::valarray<double> allCoefficients() const;
	unsigned int numTerms() const;

private:
	unsigned int nvars, order;

};


}

#endif
