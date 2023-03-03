#include "DigitMXPDisplay.h"

DigitMXPDisplay::DigitMXPDisplay() {
}

DigitMXPDisplay::~DigitMXPDisplay() {
}

bool DigitMXPDisplay::GetButtonA() {
    return m_buttonA.Get();
}

bool DigitMXPDisplay::GetButtonB() {
    return m_buttonB.Get();
}

double DigitMXPDisplay::GetPot() {
    return m_potentiometer.GetValue();
}