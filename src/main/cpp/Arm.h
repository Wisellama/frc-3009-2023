#pragma once

#include <units/angle.h>

#include <rev/RelativeEncoder.h>

#include "FeedbackController.h"

class Arm {
    private:
    // Set the arm encoder limits so that we don't attempt to move past our physical limits.
    double kEncoderLowerLimit = 0.0; // arm is in and in the robot, roughly 0 degrees straight down
    double kEncoderExtendedLowerLimit = 0.2; // extended arm down in front of robot, touching the ground, roughly 30 degrees
    double kEncoderUpperLimit = 0.6; // arm is all the way up, roughly 110 degrees

    // Set the equivalent limits in degrees. These are the physical limits of the arm in degrees.
    double kDegreesLowerLimit = 0.0;
    double kDegreesUpperLimit = 120.0;

    double kDegreesToEncoderRatio = kDegreesUpperLimit / kEncoderUpperLimit; // ratio of degress to encoder values

    double kMinimumMove = 0.01;
    double kMoveBoost = 4;

    rev::RelativeEncoder *m_encoder;
    bool m_extended = false;
    bool m_ignorelimits = false;

    FeedbackController m_feedbackController {};

    public:
    Arm(rev::RelativeEncoder *encoder);
    ~Arm();

    void ResetGoal();
    void MoveGoal(double move);
    void ToggleLimits();
    double CalculateMove();

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