#pragma once

#include <ctre/phoenix/sensors/Pigeon2.h>

#include "FeedbackController.h"

class GyroLeveller {
    private:
    ctre::phoenix::sensors::Pigeon2 *m_pigeon;

    FeedbackController m_feedbackController{};

    public:
    static constexpr double kStartingYaw = 0.0;
    static constexpr double kMaxDegrees = 180.0;

    GyroLeveller(ctre::phoenix::sensors::Pigeon2 *pigeon);
    ~GyroLeveller() {};

    double CalculateMove();

    double GetPitch();
    void SetGoal(double goal);
    void ResetGoal();
};