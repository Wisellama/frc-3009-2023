#pragma once

#include <rev/RelativeEncoder.h>

#include "FeedbackController.h"

class Wrist {
    public:
    Wrist(rev::RelativeEncoder *encoder);
    ~Wrist();

    double CalculateMove();
    void MoveGoal(double move);
    void ResetGoal();

    private:
    FeedbackController *m_feedbackController;
};
    