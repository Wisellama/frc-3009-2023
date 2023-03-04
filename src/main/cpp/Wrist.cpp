#include "Wrist.h"

Wrist::Wrist(rev::RelativeEncoder *encoder) {
    m_feedbackController = new FeedbackController(encoder);
}

Wrist::~Wrist() {
    delete m_feedbackController;
}

void Wrist::ResetGoal() {
    m_feedbackController->ResetGoal();
}

double Wrist::CalculateMove() {
    return m_feedbackController->CalculateMove();
}

void Wrist::MoveGoal(double move) {
    m_feedbackController->MoveGoal(move);
}