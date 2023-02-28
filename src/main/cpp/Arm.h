#pragma once

#include <units/angle.h>

#include <rev/SparkMaxAlternateEncoder.h>


class Arm {
    private:
    // 22.2 arm is in and in the robot, roughly 0 degrees straight down
    // 0.0 arm is all the way up, roughly 100 degrees
    // 15.0 extended arm down, roughly 30 degrees
    double kArmGoalLowerLimit = 0.0;
    double kArmGoalExtendedLowerLimit = -7.5;
    double kArmGoalUpperLimit = -23.0;
    double kMinimumMove = 0.5;
    double kGravityOffset = 1.0;

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

    units::degrees ConvertTicksToDegrees();
};