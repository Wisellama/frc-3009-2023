#include "Arm.h"

#include <algorithm>
#include <cmath>

Arm::Arm(rev::SparkMaxAlternateEncoder *encoder) {
    m_encoder = encoder;

    // Multiply our position values by some factor.
    // By default, the position value is in "rotations", so 1.0 would be a full 360 degrees.
    // If we multiply by 360, we would get degrees.
    // Alternatively you could multiply by M_PI to get radians.
    m_encoder->SetPositionConversionFactor(360);
}

void Arm::ResetGoal() {
    // Reset the arm goal to the current position
    m_goal = m_encoder->GetPosition();
}

void Arm::MoveGoal(double move) {
    m_goal += move;
}

double Arm::GetGoal() {
    return m_goal;
}

void Arm::limitGoal() {
    // The values are inverted, a lower value means a higher arm position
    double max = kArmGoalLowerLimit;
    double min = kArmGoalUpperLimit;
    if (m_extended) {
      max = kArmGoalExtendedLowerLimit;
    }

    m_goal = std::clamp(m_goal, min, max);
}

double Arm::CalculateMove() {
    double move = 0.0;

    // Make sure we clamp the goal to our physical arm limits
    limitGoal();

    // Get the current arm position. This should be in degress if our scaling factor is correct.
    m_position = m_encoder->GetPosition();

    // Figure out how far off we are from reaching the goal
    double diff = m_goal - m_position;

    // If we're close enough, don't move
    if (std::abs(diff) < kMinimumMove) {
      return 0.0;
    }

    move = diff;

    // If we're going up, add in some additional push to counteract gravity based on the arm's angle.
    if (diff > 0) {
        double gravityOffset = std::sin(m_position) * kGravityOffset;
        move += gravityOffset;
    }

    return move;
}

void Arm::SetExtended() {
    m_extended = true;
}

void Arm::SetRetracted() {
    m_extended = false;
}

void Arm::ToggleExtended() {
    m_extended = !m_extended;
}

bool Arm::GetExtendedState() {
    return m_extended;
}

double Arm::GetPosition() {
    return m_encoder->GetPosition();
}