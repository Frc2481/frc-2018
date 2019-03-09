/*
 * ComboButton.h
 *
 *  Created on: Feb 12, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMPONENTS_COMBOBUTTON_H_
#define SRC_COMPONENTS_COMBOBUTTON_H_

#include "frc/Buttons/Button.h"

class ComboButton : public frc::Button {
private:
	Button *m_primaryButton;
	Button *m_secondaryButton;
	bool m_secondaryPressed;
public:
	ComboButton(Button *primaryButton, Button *secondaryButton, bool secondaryPressed);
	virtual ~ComboButton();
	virtual bool Get();
};

#endif /* SRC_COMPONENTS_COMBOBUTTON_H_ */
