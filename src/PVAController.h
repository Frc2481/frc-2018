/*
 * PVAController.h
 *
 *  Created on: Mar 29, 2018
 *      Author: Team2481
 */

#ifndef SRC_PVACONTROLLER_H_
#define SRC_PVACONTROLLER_H_

#include <cmath>
#include "WPILib.h"

class PVAController {
private:
	double m_targetPos;
	double m_targetVel;
	double m_targetAccel;
	double m_actualPos;
	double m_actualVel;
	double m_actualAccel;
	double m_kp;
	double m_kv;
	double m_kap;
	double m_kan;
	double m_kd;
	double m_posLimMin;
	double m_posLimMax;
	double m_outLimMin;
	double m_outLimMax;
	bool m_isContinuous;
	double m_posError;
	double m_posDerError;
	double m_inputRange;
	double m_lastTime;
	double m_lastTime2;

public:
	PVAController(double kp, double kv, double kap, double kan, double kd);
	virtual ~PVAController();
	void SetGains(double kp, double kv, double kap, double kan, double kd);
	void SetActualPosition(double actualPos);
	void SetTarget(double targetPos, double targetVel, double targetAccel);
	double CalculateVelocityControlSignal();
	void SetPositionLimits(double min, double max);
	void SetOutputVelocityLimits(double minVel, double maxVel);
	void SetContinuous(bool isContinuous);
	double GetError();
	void Reset();
};

#endif /* SRC_PVACONTROLLER_H_ */
