#ifndef pargo_BoundsPair_h__guard
#define pargo_BoundsPair_h__guard


#include <vector>

namespace pargo {


class BoundsPair {

	std::pair<double, double> boundsPair;

public:

	BoundsPair ( const std::pair<double, double>& boundsPair );
				
	double lowerBound() const;
	
	double upperBound() const;
	
	operator std::pair<double, double>() const;
	
	operator std::pair<double, double>&();

};

}

#endif
