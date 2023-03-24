#include "Arm.h"

#include <algorithm>
#include <cmath>

#include <rev/RelativeEncoder.h>

#include "FeedbackController.h"

Arm::Arm(rev::RelativeEncoder *encoder) {
    m_encoder = encoder;
}

Arm::~Arm() {
}

void Arm::MoveGoal(double move) {
    m_feedbackController.MoveGoal(move);
}
void Arm::SetGoal(double move) {
    m_feedbackController.SetGoal(move);
}
void Arm::ResetGoal() {
    SetGoal(GetEncoderPosition());
}

double Arm::GetGoal() {
    return m_feedbackController.GetGoal();
}

double Arm::CalculateMove() {
    if (!m_ignorelimits) {
        m_feedbackController.ClampGoal(EncoderLowerLimit(), EncoderUpperLimit());
    }

    double move = m_feedbackController.CalculateMove(GetEncoderPosition());
    move /= kEncoderUpperLimit;
    double minCheck = 0.05;
    double minMove = 0.1;
    if (m_extended) {
        minMove *= 2;
    }
    if (std::abs(move) > minCheck) {
        if (std::abs(move) < minMove) {
            double output = minMove;
            if (move < 0) {
                output *= -1;
            }
            return output;
        } else {
            return move;
        }
    } else {
        return 0.0;
    }
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

 bool Arm::GetIgnoreLimits() {
    return m_ignorelimits;
 }

 double Arm::GetGoalWithOffset(double offset) {
if(m_ignorelimits){
    return GetGoal();
}
  if (GetGoal() > kEncoderUpperLimit) {
    return kEncoderUpperLimit - offset;
  } else if (GetGoal() <= kEncoderLowerLimit) {
    return kEncoderLowerLimit + offset;
  } else {
    return GetGoal();
  }
}