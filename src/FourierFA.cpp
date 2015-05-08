#include "FourierFA.h"


#include <cmath>

using namespace std;

namespace pargo {
	
FourierFA::FourierFA(const std::vector<BoundsPair> &bounds,
                     const FourierCoefficientGenerator& gen) :
    lb(bounds.size()),
    ub(bounds.size()),
    multipliers(gen.allCoefficients()),
    nterms(gen.numTerms()) {

    for(int i=0, size = bounds.size(); i<size; ++i) {
        lb[i] = bounds[i].lowerBound();
        ub[i] = bounds[i].upperBound();
    }

}


vector<double> FourierFA::computeFeatures ( const std::vector<double>& obs ) const{

    valarray<double> ob(&obs[0],obs.size());

    //scale observations

    ob = ((ob - lb) / (ub - lb));

    //multiply the matrices
    //[ phi ] = M_PI * [ ob ]  [              ]
    //				           [ multipliers  ]
    //                         [              ]
    // where phi and ob are 1 x nterms
    //and  multipliers is nvars x nterms

    vector<double> phi;
    phi.reserve(nterms);

    //for each column
    for(int i=0; i<nterms; ++i) {

		//inner product of observation by each column
        valarray<double> row(ob);

        //slice multipliers's column
        //and take in-place multiplication
        row *= multipliers[slice(i*ob.size(),ob.size(),1)];

        //row.sum() is the inner product
        phi.push_back(cos( M_PI * row.sum()));
    }

    return phi;
}


unsigned int FullFourierCoefficientGenerator::numTerms() const {
	return pow(order+1,nvars);
}

std::valarray<double> FullFourierCoefficientGenerator::
			allCoefficients() const {
	
	int base = order+1;
	int nterms = numTerms();


	valarray<double> multipliers(nterms*nvars);
	
	for(int row = nvars-1, frequency = 1; row>=0; --row, frequency*= base) {
		
		for(int column=0; column< nterms; ++column) {
			multipliers[column*nvars + row] = (column/frequency) % base;
		}
	}
	
	return multipliers;
}

unsigned int IndependentFourierCoefficientGenerator::numTerms() const {
	return order *nvars + 1;
}

std::valarray<double> IndependentFourierCoefficientGenerator::
			allCoefficients() const {

	valarray<double> multipliers(numTerms()*nvars);
	
	for(int row = 0; row<nvars; ++row) {
		
		for(int column=(row*order)+1,count=1; count<= order; ++count, ++column) {
			multipliers[column*nvars + row] = count;
		}
	}
	
	return multipliers;
}

}
