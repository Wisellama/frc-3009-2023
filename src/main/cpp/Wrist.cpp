#include "Wrist.h"

Wrist::Wrist() {
}

Wrist::~Wrist() {
}

void Wrist::ResetGoal() {
    SetGoal(GetPotPos());
}

double Wrist::CalculateMove() {
    //m_feedbackController.ClampGoal(kPotUpperLimit, kPotLowerLimit);

    double move = m_feedbackController.CalculateMove(GetPotPos());

    if (std::abs(move) < kMinimumMove) {
        return 0.0;
    }

    return -1 * move;
}

void Wrist::MoveGoal(double move) {
    m_feedbackController.MoveGoal(move);
}

void Wrist::SetGoal(double goal) {
    m_feedbackController.SetGoal(goal);
}

double Wrist::GetGoal() {
    return m_feedbackController.GetGoal();
}

double Wrist::GetGoalWithOffset(double offset) {
  if (GetGoal() > kPotLowerLimit) {
    return kPotLowerLimit - offset;
  } else if (GetGoal() <= kPotUpperLimit) {
    return kPotUpperLimit + offset;
  } else {
    return GetGoal();
  }
}

double Wrist::GetPotPos() {
    return m_pot.Get();
}

double Wrist::DegreesToPot(double degrees) {
    return degrees / kDegreesToPotRatio;
}
double Wrist::PotToDegrees(double pot) {
    return pot * kDegreesToPotRatio;
}