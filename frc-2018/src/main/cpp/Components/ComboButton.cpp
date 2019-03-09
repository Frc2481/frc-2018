/*
 * ComboButton.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: FIRSTMentor
 */

#include "Components/ComboButton.h"

ComboButton::ComboButton(Button *primaryButton, Button *secondaryButton, bool secondaryPressed) {
	m_primaryButton = primaryButton;
	m_secondaryButton = secondaryButton;
	m_secondaryPressed = secondaryPressed;
}

ComboButton::~ComboButton() {
	// TODO Auto-generated destructor stub
}

bool ComboButton::Get() {
	return m_primaryButton->Get() &&
		 ((m_secondaryButton->Get() && m_secondaryPressed) ||
		 (!m_secondaryButton->Get() && !m_secondaryPressed));
}
