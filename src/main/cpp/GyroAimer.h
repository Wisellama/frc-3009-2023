#pragma once

#include <ctre/phoenix/sensors/Pigeon2.h>
#include <frc/controller/PIDController.h>

class GyroAimer {
    private:
    ctre::phoenix::sensors::Pigeon2 *m_pigeon;

    const double GYRO_AIMER_P = 1.0;
    const double GYRO_AIMER_I = 0.0;
    const double GYRO_AIMER_D = 0.1;
    frc::PIDController m_pid{GYRO_AIMER_P, GYRO_AIMER_I, GYRO_AIMER_D};

    public:
    GyroAimer(ctre::phoenix::sensors::Pigeon2 *pigeon);
    ~GyroAimer() {};

    double CalculateMove(double goal);
    double CalculateToFaceStartingAngle();
};