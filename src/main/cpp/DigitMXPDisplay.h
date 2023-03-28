#pragma once

#include <frc/I2C.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>

class DigitMXPDisplay {

    public:
    static constexpr double kPotCenter = 2000.0;
    DigitMXPDisplay();
    ~DigitMXPDisplay();

    bool GetButtonA();
    bool GetButtonB();
    double GetPot();
    void InitDisplays();
    void Test();

    private:
    frc::DigitalInput m_buttonA {19};
    frc::DigitalInput m_buttonB {20};
    frc::AnalogInput m_potentiometer{7};
    //center is about 2000
    //left is about 4000
    //right is about 3-5

    frc::I2C m_i2c{frc::I2C::kMXP, 0b01110000};
};