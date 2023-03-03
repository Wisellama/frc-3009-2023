#include "Arm.h"

#include <algorithm>
#include <cmath>

Arm::Arm(rev::SparkMaxAlternateEncoder *encoder) {
    m_encoder = encoder;

    // Multiply our position values by some factor.
    // By default, the position value is in "rotations", so 1.0 would be a full 360 degrees.
    // If we multiply by 360, we would get degrees.
    // Alternatively you could multiply by M_PI to get radians.
    // m_encoder->SetPositionConversionFactor(360);
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
    double min = EncoderLowerLimit();
    double max = EncoderUpperLimit();

    m_goal = std::clamp(m_goal, min, max);
}

double Arm::CalculateMove() {
    double move = 0.0;

    // Make sure we clamp the goal to our physical arm limits
    limitGoal();

    // Get the current arm position from the encoder
    m_position = GetEncoderPosition();

    // Figure out how far off we are from reaching the goal
    double diff = m_position - m_goal;

    // If we're close enough, don't move
    if (std::abs(diff) < kMinimumMove) {
      return 0.0;
    }

    move = diff * kMoveBoost;

    // If we're going up, add in some additional push to counteract gravity based on the arm's angle.
    if (diff > 0) {
        double armAngle = EncoderToDegrees(m_position);
        double gravityOffset = std::sin(armAngle) * kGravityOffset;
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

double Arm::EncoderLowerLimit() {
    if (m_extended) {
        return kEncoderExtendedLowerLimit;
    } else {
        return kEncoderLowerLimit;
    }
}

double Arm::EncoderUpperLimit() {
    return kEncoderUpperLimit;
}

double Arm::GetEncoderPosition() {
    return m_encoder->GetPosition();
}

double Arm::DegreesToEncoder(double angle) {
    return angle / kDegreesToEncoderRatio;
}

double Arm::EncoderToDegrees(double value) {
    return value * kDegreesToEncoderRatio;
}