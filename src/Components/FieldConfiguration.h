/*
 * FieldConfiguration.h
 *
 *  Created on: Jan 15, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_FIELDCONFIGURATION_H_
#define SRC_FIELDCONFIGURATION_H_

#include <map>
#include "DriverStation.h"

class FieldConfiguration {
public:

	enum Plate {
		LEFT,
		RIGHT,
		UNKNOWN
	};

	FieldConfiguration();
	virtual ~FieldConfiguration();

	void Initialize();
	Plate GetOurSwitchPlate();
	Plate GetTheirSwitchPlate();
	Plate GetScalePlate();
private:
	Plate m_ourSwitchPlate;
	Plate m_theirSwitchPlate;
	Plate m_scalePlate;
	std::map<char, Plate> m_plateMap = {
			{'L', LEFT},
			{'R', RIGHT}
	};
	Plate LookupPlate(char c);
};

#endif /* SRC_FIELDCONFIGURATION_H_ */
