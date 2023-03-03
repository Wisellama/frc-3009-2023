#pragma once

#include <units/angle.h>

#include <rev/SparkMaxAlternateEncoder.h>


class Arm {
    private:
    // Set the arm encoder limits so that we don't attempt to move past our physical limits.
    double kEncoderLowerLimit = 0.0; // arm is in and in the robot, roughly 0 degrees straight down
    double kEncoderExtendedLowerLimit = 0.197; // extended arm down in front of robot, touching the ground, roughly 30 degrees
    double kEncoderUpperLimit = 0.57; // arm is all the way up, roughly 110 degrees

    // Set the equivalent limits in degrees. These are the physical limits of the arm in degrees.
    double kDegreesLowerLimit = 0.0;
    double kDegreesUpperLimit = 120.0;

    double kDegreesToEncoderRatio = kDegreesUpperLimit / kEncoderUpperLimit; // ratio of degress to encoder values

    double kMinimumMove = 0.01;
    //double kGravityOffset = 1 / kDegreesToEncoderRatio;
    double kGravityOffset = 0; // TODO enable this after testing
    double kMoveBoost = 2;

    rev::SparkMaxAlternateEncoder *m_encoder;
    double m_goal = 0.0;
    double m_position = 0.0;
    bool m_extended = false;

    void limitGoal();

    public:
    Arm(rev::SparkMaxAlternateEncoder *encoder);
    ~Arm() {};

    void ResetGoal();
    void MoveGoal(double move);
    double CalculateMove();
    double GetPosition();
    double GetGoal();

    void SetExtended();
    void SetRetracted();
    void ToggleExtended();
    bool GetExtendedState();

    double EncoderLowerLimit();
    double EncoderUpperLimit();
    double GetEncoderPosition();
    double DegreesToEncoder(double angle);
    double EncoderToDegrees(double value);
};