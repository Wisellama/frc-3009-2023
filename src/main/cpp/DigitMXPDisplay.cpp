#include "DigitMXPDisplay.h"

#include <chrono>
#include <thread>

DigitMXPDisplay::DigitMXPDisplay() {
    // Supposed to wait for 1ms before sending data.
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // Setup displays
    uint8_t setup[] = {0b00100001};
    m_i2c.WriteBulk(setup, 1);

    // Set ROW driver output
    //m_i2c.WriteBulk(0b10100000);

    // Turn displays on, no blinking
    uint8_t on[] = {0b10000001};
    m_i2c.WriteBulk(on, 1);
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


void DigitMXPDisplay::Test() {
    uint8_t d[] = {0, 0b11101001};
    m_i2c.WriteBulk(d, 2);

    uint8_t a[] = {0, 0b11101101};
    m_i2c.WriteBulk(a, 2);

    uint8_t c[] = {0, 0b11101010};
    m_i2c.WriteBulk(c, 2);

    uint8_t f[] = {0, 0b11100001};
    m_i2c.WriteBulk(f, 2);
}
