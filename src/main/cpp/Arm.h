#pragma once

#include <units/angle.h>

#include <rev/RelativeEncoder.h>

#include "FeedbackController.h"

class Arm {
    private:
    double kMinimumMove = 0.01;
    double kMoveBoost = 4;

    rev::RelativeEncoder *m_encoder;
    bool m_extended = false;
    bool m_ignorelimits = false;

    FeedbackController m_feedbackController {};

    public:
    // Set the arm encoder limits so that we don't attempt to move past our physical limits.
    static constexpr double kEncoderLowerLimit = 0.0; // arm is in and in the robot, roughly 0 degrees straight down
    static constexpr double kEncoderExtendedLowerLimit = 0.2; // extended arm down in front of robot, touching the ground, roughly 30 degrees
    static constexpr double kEncoderUpperLimit = 0.6; // arm is all the way up, roughly 110 degrees

    // Set the equivalent limits in degrees. These are the physical limits of the arm in degrees.
    static constexpr double kDegreesLowerLimit = 0.0;
    static constexpr double kDegreesUpperLimit = 120.0;

    static constexpr double kDegreesToEncoderRatio = kDegreesUpperLimit / kEncoderUpperLimit; // ratio of degress to encoder values

    Arm(rev::RelativeEncoder *encoder);
    ~Arm();

    void ResetGoal();
    void MoveGoal(double move);
    void ToggleLimits();
    double CalculateMove();
    void SetGoal(double move);
    void SetExtended();
    void SetRetracted();
    void ToggleExtended();
    bool GetExtendedState();
    bool GetIgnoreLimits();

    double EncoderLowerLimit();
    double EncoderUpperLimit();
    double GetEncoderPosition();
    double DegreesToEncoder(double angle);
    double EncoderToDegrees(double value);
};