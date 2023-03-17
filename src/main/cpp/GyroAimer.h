#pragma once

#include <ctre/phoenix/sensors/Pigeon2.h>

#include "FeedbackController.h"

class GyroAimer {
    private:
    ctre::phoenix::sensors::Pigeon2 *m_pigeon;

    FeedbackController m_feedbackController{};

    public:
    static constexpr double kStartingYaw = -90.0; // For whatever reason, the gyro starts at -90 degrees.
    static constexpr double kMaxDegrees = 180.0;

    GyroAimer(ctre::phoenix::sensors::Pigeon2 *pigeon);
    ~GyroAimer() {};

    double CalculateMove();
    double CalculateToFaceStartingAngle();

    double GetYaw();
    void SetGoal(double goal);
    void ResetGoal();
};