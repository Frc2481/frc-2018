/*
 * RollingAccumulator.h
 *
 *  Created on: Jan 24, 2014
 *      Author: Team2481
 */

#ifndef ROLLINGACCUMULATOR_H_
#define ROLLINGACCUMULATOR_H_

template<class A, int I>
class RollingAccumulator {
private:
	A _values[I];
	A _sum;
	A _sumSqr;
	int _index;

public:
	RollingAccumulator() {
		_index = 0;
		reset();
	}
	virtual ~RollingAccumulator() {}

	void reset() {
		_sumSqr = _sum = _index = 0;
		for (int i = 0; i < I; ++i) {
			_values[i] = 0;
		}
		//memset(_values, 0, sizeof(A) * I);
	}

	void add(A v) {
		_sum -= _values[_index];
		_sumSqr -= _values[_index] * _values[_index];
		_values[_index] = v;
		_sum += v;
		_sumSqr += (v * v);
		_index++;
		_index = _index % I;
	}

	A avg() {
		return _sum / I;
	}

	A stddev(){
		return sqrt((_sumSqr - (_sum * _sum)/ I)/ I);
	}
	A recentVal(){
		return _values[(_index + I - 1) % I];
	}
};

#endif /* ROLLINGACCUMULATOR_H_ */
