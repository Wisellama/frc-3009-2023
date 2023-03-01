#include "GyroAimer.h"

#include <frc/ADIS16448_IMU.h>
#include <frc/controller/PIDController.h>

GyroAimer::GyroAimer(frc::ADIS16448_IMU *imu) {
    m_imu = imu;
}

double GyroAimer::CalculateMove(double goal) {
    double current = m_imu->GetAngle().value();

    return m_pid.Calculate(current, goal);
}

double GyroAimer::CalculateToFaceStartingAngle() {
    double goal = 0.0;
    double current = m_imu->GetAngle().value();

    return m_pid.Calculate(current, goal);
}