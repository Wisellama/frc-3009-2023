#include "GyroAimer.h"

#include <ctre/phoenix/sensors/Pigeon2.h>
#include <frc/controller/PIDController.h>
#include <algorithm>
#include <math.h>

GyroAimer::GyroAimer(ctre::phoenix::sensors::Pigeon2 *pigeon) {
    m_pigeon = pigeon;
}

double GyroAimer::CalculateMove(double goal) {
    double current = m_pigeon->GetYaw();

    return m_pid.Calculate(current, goal);
}

double GyroAimer::CalculateToFaceStartingAngle() {
    // imu calibrates and starts at -90
    // this will probably break if the gyro is ever calibrated again for some reason
    double goal = -90;
    double current = m_pigeon->GetYaw();
    goal = std::floor(current / 360.0) + goal;

    return std::clamp(m_pid.Calculate(current, goal), -1.0, 1.0);
}