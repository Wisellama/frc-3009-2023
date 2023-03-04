#include "FeedbackController.h"

#include <algorithm>

FeedbackController::FeedbackController(rev::RelativeEncoder *encoder) {
    m_encoder = encoder;
}

void FeedbackController::ResetGoal() {
    m_goal = GetEncoderPosition();
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

double FeedbackController::CalculateMove() {
    // Get the current arm position from the encoder
    m_position = GetEncoderPosition();

    // Figure out how far off we are from reaching the goal
    double diff = m_position - m_goal;

    return diff;
}

double FeedbackController::GetEncoderPosition() {
    return m_encoder->GetPosition();
}