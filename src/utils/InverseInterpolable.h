#pragma once
template <class T>
class InverseInterpolable {
public:
	virtual double inverseInterpolate(const T &upper, const T &query) const = 0 ;
};