#pragma once

#include <frc/I2C.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>

class DigitMXPDisplay {
    public:
    DigitMXPDisplay();
    ~DigitMXPDisplay();

    bool GetButtonA();
    bool GetButtonB();
    double GetPot();

    private:
    frc::DigitalInput m_buttonA {9};
    frc::DigitalInput m_buttonB {10};
    frc::AnalogInput m_potentiometer{3};    
};