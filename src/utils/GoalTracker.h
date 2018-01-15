#pragma once
#include "utils/Translation2D.h"
#include "utils/GoalTrack.h"
#include "utils/RigidTransform2D.h"
#include <set>
#include <list>

class GoalTracker {
public:
	class TrackReport {
	public:
		RigidTransform2D m_fieldToGoal;
		double m_latestTimestamp;
		double m_stability;
		int m_id;
		double m_LastTrackId;
		TrackReport(const GoalTrack &track);
		//double score() const;
		friend bool operator<(const TrackReport &lhs, const TrackReport &rhs);
	};
private:
	std::list<GoalTrack> m_currentTracks;
	int m_nextId = 0;
public:
	GoalTracker();
	void reset();
	void update(double timestamp, RigidTransform2D &fieldToGoals);
	bool hasTracks();
	std::set<TrackReport> getTracks();
};
