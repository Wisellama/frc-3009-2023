#include "GyroAimer.h"

#include <frc/ADIS16448_IMU.h>
#include <frc/controller/PIDController.h>
#include <algorithm>
#include <math.h>

GyroAimer::GyroAimer(frc::ADIS16448_IMU *imu) {
    m_imu = imu;
}

double GyroAimer::CalculateMove(double goal) {
    double current = m_imu->GetAngle().value();

    return m_pid.Calculate(current, goal);
}

double GyroAimer::CalculateToFaceStartingAngle() {
    // imu calibrates and starts at -90
    // this will probably break if the gyro is ever calibrated again for some reason
    double goal = -90;
    double current = m_imu->GetAngle().value();
    goal = std::floor(current / 360.0) + goal;

    return std::clamp(m_pid.Calculate(current, goal), 0.0, 1.0);
}