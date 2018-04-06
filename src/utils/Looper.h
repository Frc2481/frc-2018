/*
 * Looper.h
 *
 *  Created on: Jan 30, 2017
 *      Author: Team2481
 */

#ifndef SRC_COMPONENTS_LOOPER_H_
#define SRC_COMPONENTS_LOOPER_H_
#include "thread"
#include "atomic"
#include "Utility.h"

class Looper {
protected:
	std::atomic<bool> m_active;
	std::thread m_thread;
	int m_interval;
	bool m_started;
public:
	Looper();
	Looper(int interval);
	virtual ~Looper();
	void Start();
	virtual void OnStart() = 0;
	virtual void OnLoop() = 0;
	virtual void OnStop() = 0;
	void Loop();
	void SetActive(bool active);
};

#endif /* SRC_COMPONENTS_LOOPER_H_ */
