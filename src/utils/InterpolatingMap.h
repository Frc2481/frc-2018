#pragma once
#include <map>
#include "utils/InverseInterpolable.h"
#include "utils/Interpolable.h"

template <class K, class V>
class InterpolatingMap : public std::map<K, V> {
public:
	InterpolatingMap(size_t maximumSize) {
		m_max = maximumSize;
	}

	InterpolatingMap() {
		m_max = 0;
	}

	V put(K key, V value) {
		if (m_max > 0 && m_max <= std::map<K, V>::size()) {
			auto first = std::map<K, V>::begin();
			std::map<K, V>::erase(first);
		}

		std::map<K, V>::emplace(key, value);

		return value;
	}

	V getInterpolated(K key) {
		auto gotval = std::map<K, V>::find(key);
		if (gotval == std::map<K, V>::end()) {
			auto topBound = std::map<K, V>::upper_bound(key);
			auto bottomBound = std::map<K, V>::lower_bound(key);
			bottomBound--;

			if (topBound == std::map<K, V>::end() && bottomBound == std::map<K, V>::end()) {
				return std::map<K, V>::end()->second;
			}
			else if (topBound == std::map<K, V>::end()) {
				return bottomBound->second;
			}
			else if (bottomBound == std::map<K, V>::end()) {
				return topBound->second;
			}

			V topElem = topBound->second;
			V bottomElem = bottomBound->second;
			return bottomElem.interpolate(topElem, bottomBound->first.inverseInterpolate(topBound->first, key));
		}
		else {
			return gotval->second;
		}
	}

private:	
	size_t m_max;
};
