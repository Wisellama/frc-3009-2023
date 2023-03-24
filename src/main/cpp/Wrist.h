#pragma once

#include <frc/AnalogPotentiometer.h>

#include "FeedbackController.h"

class Wrist {
    public:
    // Set the limits so that we don't attempt to move past our physical limits.
    // NOTE: the values are reversed, going up means lower values. All the way up is 0. All the way down is 1.
    static constexpr double kPotUpperLimit = 0.1; // wrist is all the way up (how it sits when the arm is in the robot)
    static constexpr double kPotLevel = 0.6; // not quite linear to go to 90 degrees level
    static constexpr double kPotLowerLimit = 1.0; // wrist is all the way down (picking up a cube in front of the robot)

    // Set the equivalent limits in degrees. These are the physical limits of the arm in degrees.
    static constexpr double kDegreesLowerLimit = 0.0;
    static constexpr double kDegreesUpperLimit = 180.0;

    static constexpr double kDegreesToPotRatio = kDegreesUpperLimit / kPotLowerLimit; // ratio of degress to potentiometer values

    Wrist();
    ~Wrist();

    double CalculateMove();
    void MoveGoal(double move);
    void ResetGoal();
    void SetGoal(double goal);
    double GetGoal();
    double GetGoalWithOffset(double offset);

    double GetPotPos();
    double DegreesToPot(double degrees);
    double PotToDegrees(double pot);

    private:
    const double kMinimumMove = 0.1;

    FeedbackController m_feedbackController {};
    frc::AnalogPotentiometer m_pot {0};
};
    