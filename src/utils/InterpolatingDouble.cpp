#include "utils/InterpolatingDouble.h"

InterpolatingDouble::InterpolatingDouble(double val) {
	m_value = val;
}

InterpolatingDouble InterpolatingDouble::interpolate(const InterpolatingDouble &other, double x) const{
	double dydx = other.m_value - m_value;
	double searchY = dydx * x + m_value;
	return InterpolatingDouble(searchY);
}

double InterpolatingDouble::inverseInterpolate(const InterpolatingDouble &upper, const InterpolatingDouble &query) const {
	double upper_to_lower = upper.m_value - m_value;
	if (upper_to_lower <= 0) {
		return 0;
	}
	double query_to_lower = query.m_value - m_value;
	if (query_to_lower <= 0) {
		return 0;
	}
	return query_to_lower / upper_to_lower;
}

bool operator<(const InterpolatingDouble &lhs, const InterpolatingDouble &rhs) {
	return lhs.m_value < rhs.m_value;
}

bool InterpolatingDouble::operator==(const InterpolatingDouble & other) {
	return m_value == other.m_value;
}
