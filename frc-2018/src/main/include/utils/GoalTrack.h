#pragma once
#include "utils/RigidTransform2D.h"
#include <map>

class GoalTrack {
private: 
	std::map<double, RigidTransform2D> m_observePositions;
	RigidTransform2D m_smoothPosition;
	int m_id;
	
public:
	GoalTrack(double timestamp, const RigidTransform2D &firstObservation, int id);
	void pruneByTime(double timestamp);
	bool tryUpdate(double timestamp, const RigidTransform2D &newObservation);
	bool isAlive() const;
	void smooth();
	RigidTransform2D getSmoothedPosition() const;
	double getLatestTimestamp() const;
	double getStability() const;
	int getId() const;
};
