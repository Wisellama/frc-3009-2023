#include "Wrist.h"

Wrist::Wrist(rev::RelativeEncoder *encoder) {
    m_encoder = encoder;
}

Wrist::~Wrist() {
}

void Wrist::ResetGoal() {
    m_feedbackController.SetGoal(GetEncoderPosition());
}

double Wrist::CalculateMove() {
    double move = m_feedbackController.CalculateMove(GetEncoderPosition());

    if (std::abs(move) < kMinimumMove) {
        return 0.0;
    }

    return move;
}

void Wrist::MoveGoal(double move) {
    m_feedbackController.MoveGoal(move);
}

double Wrist::GetEncoderPosition() {
    return m_encoder->GetPosition();
}

double Wrist::DegreesToEncoder(double degrees) {
    return degrees / kDegreesToEncoderRatio;
}
double Wrist::EncoderToDegrees(double encoder) {
    return encoder * kDegreesToEncoderRatio;
}