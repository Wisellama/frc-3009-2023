#include "FeedbackController.h"

#include <algorithm>

FeedbackController::FeedbackController() {
}

void FeedbackController::SetGoal(double position) {
    m_goal = position;
}

void FeedbackController::MoveGoal(double move) {
    m_goal += move;
}

double FeedbackController::GetGoal() {
    return m_goal;
}

void FeedbackController::ClampGoal(double min, double max) {
    m_goal = std::clamp(m_goal, min, max);
}

double FeedbackController::CalculateMove(double position) {
    // Figure out how far off we are from reaching the goal
    double diff = position - m_goal;

    return diff;
}
