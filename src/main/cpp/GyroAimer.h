#pragma once

#include <frc/ADIS16448_IMU.h>
#include <frc/controller/PIDController.h>

class GyroAimer {
    private:
    frc::ADIS16448_IMU *m_imu;

    const double GYRO_AIMER_P = 1.0;
    const double GYRO_AIMER_I = 0.0;
    const double GYRO_AIMER_D = 0.1;
    frc::PIDController m_pid{GYRO_AIMER_P, GYRO_AIMER_I, GYRO_AIMER_D};

    public:
    GyroAimer(frc::ADIS16448_IMU *imu);
    ~GyroAimer() {};

    double CalculateMove(double goal);
    double CalculateToFaceStartingAngle();
};