#pragma once
template <class T>
class Interpolable {
	virtual T interpolate(const T &other, double x) const = 0;
}; 
