#include "utils/GoalTracker.h"
#include "utils/Constants.h"

GoalTracker::TrackReport::TrackReport(const GoalTrack &track)	
	: m_fieldToGoal(track.getSmoothedPosition()),
		m_latestTimestamp(track.getLatestTimestamp()),
		m_stability(track.getStability()), 
		m_id(track.getId()){
}

//double GoalTracker::TrackReport::score() const {
//	double m_StabilityScore = Constants::kStabilityWeight * m_stability;
//	double m_AgeScore = Constants::kAgeWeight
//		* max(0, Constants::kMaxGoalTrackAge - (m_CurrentTimestamp - m_latestTimestamp))
//		/ Constants::kMaxGoalTrackAge;
//	double m_SwitchingScore = (m_id == m_LastTrackId ? Constants::kSwitchingWeight : 0);
//	return m_StabilityScore + m_AgeScore + m_SwitchingScore;
//}

bool operator<(const GoalTracker::TrackReport &lhs, const GoalTracker::TrackReport &rhs) {
	double stability = (lhs.m_stability - rhs.m_stability) * Constants::kStabilityWeight;
	double age = (lhs.m_latestTimestamp - rhs.m_latestTimestamp) * Constants::kAgeWeight;
	double finalScore = stability + age;

	if (finalScore > 0) {
		return false;
	}
	else if (finalScore < 1e-5) {
		if (age > 0) {
			return false;
		}
		else if (stability > 0) {
			return false;
		}
	}
	return true;
}

void GoalTracker::reset() {
	m_currentTracks.clear();
}

void GoalTracker::update(double timestamp, RigidTransform2D &fieldToGoals) {
	//bool hasUpdatedTrack = false; 
	bool hasUpdatedTrack = false;
	for (GoalTrack track : m_currentTracks) {
		if (!hasUpdatedTrack) {
			if (track.tryUpdate(timestamp, fieldToGoals)) {
				hasUpdatedTrack = true;
			}
		}else {
			track.pruneByTime(timestamp);
		}
	}
	for (std::list<GoalTrack>::iterator it = m_currentTracks.begin(); it != m_currentTracks.end();) {
		if (!it->isAlive())
		{
			it = m_currentTracks.erase(it);
		}
		else {
			it++;
		}
	}
	if (m_currentTracks.empty()) {
		m_currentTracks.push_back(GoalTrack(timestamp, fieldToGoals, m_nextId));
		++m_nextId;
	}
}

bool GoalTracker::hasTracks() {
	return !m_currentTracks.empty();
}

std::set<GoalTracker::TrackReport> GoalTracker::getTracks() {
	std::set<GoalTracker::TrackReport> currentTrackReports(m_currentTracks.begin(),m_currentTracks.end());
	return currentTrackReports;
}

GoalTracker::GoalTracker() {
}
