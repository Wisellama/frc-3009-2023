#pragma once

#include <rev/RelativeEncoder.h>

#include "FeedbackController.h"

class Wrist {
    public:
    Wrist(rev::RelativeEncoder *encoder);
    ~Wrist();

    double CalculateMove();
    void MoveGoal(double move);
    void ResetGoal();

    double GetEncoderPosition();
    double DegreesToEncoder(double degrees);
    double EncoderToDegrees(double encoder);

    private:
    const double kMinimumMove = 0.01;
    // Set the encoder limits so that we don't attempt to move past our physical limits.
    double kEncoderLowerLimit = 0.0; // wrist is all the way up (how it sits when the arm is in the robot)
    double kEncoderUpperLimit = 1.0; // wrist is all the way down (picking up a cube in front of the robot)

    // Set the equivalent limits in degrees. These are the physical limits of the arm in degrees.
    double kDegreesLowerLimit = 0.0;
    double kDegreesUpperLimit = 180.0;

    double kDegreesToEncoderRatio = kDegreesUpperLimit / kEncoderUpperLimit; // ratio of degress to encoder values

    rev::RelativeEncoder *m_encoder;
    FeedbackController m_feedbackController {};
};
    