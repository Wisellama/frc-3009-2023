#include "DigitMXPDisplay.h"

#include <chrono>
#include <thread>

DigitMXPDisplay::DigitMXPDisplay() {
    // Supposed to wait for 1ms before sending data.
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // Setup displays
    for(int i = 0; i < 4; i++) {
        Write(i, 0b00100001);
    }

    // Set ROW driver output
    for(int i = 0; i < 4; i++) {
        Write(i, 0b10100000);
    }

    // Turn displays on, no blinking
    for(int i = 0; i < 4; i++) {
        Write(i, 0b10000001);
    }
}

DigitMXPDisplay::~DigitMXPDisplay() {
}

bool DigitMXPDisplay::GetButtonA() {
    return !m_buttonA.Get();
}

bool DigitMXPDisplay::GetButtonB() {
    return !m_buttonB.Get();
}

double DigitMXPDisplay::GetPot() {
    return m_potentiometer.GetValue();
}

void DigitMXPDisplay::Write(int registerAddress, uint8_t data) {
    m_i2c.Write(registerAddress, data);
}

void DigitMXPDisplay::Test() {
    Write(0, 0b11101001); // D
    Write(1, 0b11101101); // A
    Write(2, 0b11101010); // C
    Write(3, 0b11100001); // F
}
