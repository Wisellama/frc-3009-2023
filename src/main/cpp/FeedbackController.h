#pragma once

#include <rev/RelativeEncoder.h>

class FeedbackController {
    public:
    FeedbackController(rev::RelativeEncoder *encoder);
    ~FeedbackController(){};

    void ResetGoal();
    void MoveGoal(double move);
    double GetGoal();
    void ClampGoal(double min, double max);
    double CalculateMove();

    double GetEncoderPosition();

    private:
    double m_goal = 0.0;
    double m_position = 0.0;
    rev::RelativeEncoder *m_encoder;

};