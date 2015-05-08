#include "BoundsPair.h"

#include <stdexcept>

namespace pargo {



BoundsPair::BoundsPair ( const std::pair<double, double>& boundsPair ) :
		boundsPair ( boundsPair ) {
			if(boundsPair.first > boundsPair.second)
				throw std::invalid_argument("lower bounds higher than upper bound");
		}

double BoundsPair::lowerBound() const {
	return boundsPair.first;
}

double BoundsPair::upperBound() const {
	return boundsPair.second;
}

BoundsPair::operator std::pair<double, double>() const {
	return boundsPair;
}

BoundsPair::operator std::pair<double, double>&() {
	return boundsPair;
}

}

