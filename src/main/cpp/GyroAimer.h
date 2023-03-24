#pragma once

#include <ctre/phoenix/sensors/Pigeon2.h>

#include "FeedbackController.h"

class GyroAimer {
    private:
    ctre::phoenix::sensors::Pigeon2 *m_pigeon;

    double m_startingYaw = 0.0;

    FeedbackController m_feedbackController{};

    public:
    static constexpr double kMaxDegrees = 360.0;

    GyroAimer(ctre::phoenix::sensors::Pigeon2 *pigeon);
    ~GyroAimer() {};

    double CalculateMove();
    double CalculateToFaceStartingAngle();

    double GetYaw();
    void SetGoal(double goal);
    void SetGoalToOtherSide();
    void ResetGoal();
};