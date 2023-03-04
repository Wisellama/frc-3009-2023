#include "Arm.h"

#include <algorithm>
#include <cmath>

#include <rev/RelativeEncoder.h>

#include "FeedbackController.h"

Arm::Arm(rev::RelativeEncoder *encoder) {
    m_feedbackController = new FeedbackController(encoder);
}

Arm::~Arm() {
    delete m_feedbackController;
}

void Arm::MoveGoal(double move) {
    m_feedbackController->MoveGoal(move);
}

void Arm::ResetGoal() {
    m_feedbackController->ResetGoal();
}

double Arm::CalculateMove() {
    if (!m_ignorelimits) {
        m_feedbackController->ClampGoal(EncoderLowerLimit(), EncoderUpperLimit());
    }

    double move = m_feedbackController->CalculateMove();
    if (std::abs(move) < kMinimumMove) {
        return 0.0;
    }

    return move*kMoveBoost;
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

 void Arm::ToggleLimits(){
    m_ignorelimits = !m_ignorelimits;
 }