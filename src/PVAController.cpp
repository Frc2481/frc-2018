/*
 * PVAController.cpp
 *
 *  Created on: Mar 29, 2018
 *      Author: Team2481
 */

#include <PVAController.h>

PVAController::PVAController(double kp, double kv, double kap, double kan, double kd) {
	m_targetPos = 0;
	m_targetVel = 0;
	m_targetAccel = 0;
	m_actualPos = 0;
	m_actualVel = 0;
	m_actualAccel = 0;
	m_kp = 0;
	m_kv = 0;
	m_kap = 0;
	m_kan = 0;
	m_kd = 0;
	m_posLimMin = 0;
	m_posLimMax = 0;
	m_outLimMin = -1;
	m_outLimMax = 1;
	m_isContinuous = false;
	m_posError = 0;
	m_velError = 0;
	m_inputRange = 0;
	m_lastTime = 0;
}

PVAController::~PVAController() {
}

void PVAController::SetGains(double kp, double kv, double kap, double kan, double kd) {
	m_kp = kp;
	m_kv = kv;
	m_kap = kap;
	m_kan = kan;
	m_kd = kd;
}

void PVAController::SetActualPosition(double actualPos) {
	double newTime = RobotController::GetFPGATime();
	double dt = newTime - m_lastTime;
	m_actualVel = (m_actualPos - actualPos) / dt;
	m_actualPos = actualPos;
	m_lastTime = newTime;
}

void PVAController::SetTarget(double targetPos, double targetVel, double targetAccel) {
	m_targetPos = targetPos;
	m_targetVel = targetVel;
	m_targetAccel = targetAccel;
}

double PVAController::CalculateVelocityControlSignal() {
	// calculate position error
	m_posError = m_targetPos - m_actualPos;

	// calculate velocity error
	m_velError = m_targetVel - m_actualVel;

	// wraparound position error
//	if(m_isContinuous) {
//		if(m_error > (m_posLimMax - (m_posLimMax - m_posLimMin) / 2)) {
//			m_error = -(m_posLimMax - m_targetPos) + (m_posLimMin - m_actualPos);
//		}
//		else if(m_error < -(m_posLimMax - (m_posLimMax - m_posLimMin) / 2)) {
//			m_error = -(m_posLimMin - m_targetPos) + (m_posLimMax - m_actualPos);
//		}

	if (m_isContinuous && m_inputRange != 0) {
		m_posError = std::fmod(m_posError, m_inputRange);
		if (std::fabs(m_posError) > m_inputRange / 2) {
			if (m_posError > 0) {
				return m_posError - m_inputRange;
			} else {
				return m_posError + m_inputRange;
			}
		}
	}

	// calculate velocity control signal
	double velControl = 0;
//	if(m_targetAccel > 1) {
		velControl = m_kap * m_targetAccel + m_kv * m_targetVel + m_kp * m_posError + m_kd * m_velError;
//	}
//	else {
//		velControl = m_kan * m_targetAccel + m_kv * m_targetVel + m_kp * m_posError + m_kd * m_velError;
//	}

	// limit velocity control signal
	if(velControl > m_outLimMax) {
		velControl = m_outLimMax;
	}
	else if(velControl < m_outLimMin) {
		velControl = m_outLimMin;
	}

	return velControl;
}

void PVAController::SetPositionLimits(double minPos, double maxPos) {
	m_posLimMin = minPos;
	m_posLimMax = maxPos;
    m_inputRange = maxPos - minPos;
}

void PVAController::SetOutputVelocityLimits(double minVel, double maxVel) {
	m_outLimMin = minVel;
	m_outLimMax = maxVel;

}

void PVAController::SetContinuous(bool isContinuous) {
	m_isContinuous = isContinuous;
}

double PVAController::GetError() {
	return m_posError;
}

void PVAController::Reset() {
	m_posError = 0;
	m_targetPos = 0;
	m_targetVel = 0;
	m_targetAccel = 0;
}